import pytest

from overwatch.decision import FALSE_ALARM_BELOW, combine
from overwatch.decision.laya_decider import LayaDecider
from overwatch.decision.questions import QUESTIONS, action_from_answers, expected_from_verdict
from overwatch.decision.rules import RulesDecider
from overwatch.events import CAMERA_FAULT, DROPPED_ITEM, Decision, Observation
from overwatch.store import IncidentStore


def drop(area=0.08, **f):
    return Observation(DROPPED_ITEM, "table", 1.0, "x", {"area_pct": area, "solidity": 0.95, "aspect": 1.3, **f})


def test_small_item_offers_recovery():
    d = RulesDecider().decide(drop())
    assert d.action == "propose_recovery" and d.recoverable


def test_too_big_for_one_item_goes_to_a_person():
    d = RulesDecider().decide(drop(area=3.0))
    assert d.action == "escalate" and d.severity == 2


def test_camera_fault_escalates():
    d = RulesDecider().decide(Observation(CAMERA_FAULT, "camera", 1.0, "x", {"fault": "black"}))
    assert d.action == "escalate"


def test_two_questions_make_the_action():
    assert action_from_answers(False, True) == "ignore"
    assert action_from_answers(True, True) == "propose_recovery"
    assert action_from_answers(True, False) == "escalate"


def _m(action, p_real, name="laya"):
    return Decision(action, 1, action == "propose_recovery", [f"{name}: {action}"], name, p_recoverable=0.5,
                    latency_ms=12.0, extra={"p_real_drop": p_real})


def test_shadow_never_changes_the_rules_decision():
    rules = RulesDecider().decide(drop())
    assert combine(rules, _m("ignore", 0.01), "shadow") is rules
    assert combine(rules, _m("ignore", 0.01), "off") is rules


def test_filter_drops_a_confident_false_alarm():
    rules = RulesDecider().decide(drop())
    out = combine(rules, _m("ignore", FALSE_ALARM_BELOW - 0.05), "filter")
    assert out.action == "ignore" and "false alarm" in out.reasons[0]


def test_filter_keeps_an_unsure_case_and_a_confident_real_drop():
    rules = RulesDecider().decide(drop())
    assert combine(rules, _m("propose_recovery", 0.6), "filter") is rules
    assert combine(rules, _m("ignore", FALSE_ALARM_BELOW + 0.1), "filter") is rules      # not sure: keep alerting


def test_filter_can_hand_an_unrecoverable_real_drop_to_a_person():
    rules = RulesDecider().decide(drop())
    out = combine(rules, _m("escalate", 0.9), "filter")
    assert out.action == "escalate" and not out.recoverable


def test_filter_never_touches_camera_faults():
    rules = RulesDecider().decide(Observation(CAMERA_FAULT, "camera", 1.0, "x", {"fault": "black"}))
    assert combine(rules, _m("ignore", 0.0), "filter") is rules


def test_laya_answer_mapping():
    ans = {"real_drop": {"noul": 0.8, "confidence": 0.8}, "recoverable": {"noul": 0.7, "confidence": 0.7}}
    d = LayaDecider._to_decision(ans, 40.0)
    assert d.action == "propose_recovery" and d.extra["p_real_drop"] == 0.8 and d.recoverable
    d = LayaDecider._to_decision({"real_drop": {"noul": 0.1, "confidence": 0.9},
                                  "recoverable": {"noul": 0.7, "confidence": 0.7}}, 40.0)
    assert d.action == "ignore"
    assert LayaDecider._to_decision({}, 1.0) is None


def test_laya_is_asked_exactly_the_two_questions():
    class FakeRouter:
        calls = []

        def predict(self, state, questions, model=None):
            FakeRouter.calls.append((state, questions, model))
            return {"answers": {"real_drop": {"noul": 0.9, "confidence": 0.9},
                                "recoverable": {"noul": 0.8, "confidence": 0.8}}}

    laya = LayaDecider("typed-decisions")
    laya._router, laya.available = FakeRouter(), True
    d = laya.decide(drop())
    assert d.action == "propose_recovery"
    state, questions, model = FakeRouter.calls[0]
    assert questions is QUESTIONS and set(questions) == {"real_drop", "recoverable"} and model == "typed-decisions"
    assert set(state) == {"report"} and "tiny object appeared on the table" in state["report"]


def test_laya_load_failure_degrades_quietly(monkeypatch):
    import builtins
    real = builtins.__import__

    def fake_import(name, *a, **k):
        if name == "laya":
            raise ImportError("No module named 'laya'")
        return real(name, *a, **k)
    monkeypatch.setattr(builtins, "__import__", fake_import)
    laya = LayaDecider()
    assert laya.decide(drop()) is None and not laya.available


def test_verdicts_become_labels():
    assert expected_from_verdict("approve_recovery") == {"real_drop": True, "recoverable": True}
    assert expected_from_verdict("escalate") == {"real_drop": True, "recoverable": False}
    assert expected_from_verdict("dismiss") == {"real_drop": False}
    assert expected_from_verdict("nonsense") == {}


def test_store_filtered_are_kept_but_not_listed_and_agreement_counts():
    store = IncidentStore(":memory:")
    o = drop()
    rules = RulesDecider().decide(o)
    laya = _m("escalate", 0.9)
    a = store.add(kind=o.kind, zone=o.zone, track_id=3, observation=o.to_dict(), final=rules.to_dict(),
                  rules=rules.to_dict(), laya=laya.to_dict(), snapshot=None)
    store.add(kind=o.kind, zone=o.zone, track_id=4, observation=o.to_dict(), final=rules.to_dict(),
              rules=rules.to_dict(), laya=None, snapshot=None, status="filtered")
    assert [i["id"] for i in store.list()] == [a["id"]]
    assert len(store.list("filtered")) == 1 and len(store.list("open")) == 1
    store.set_verdict(a["id"], "approve_recovery", "", None, "actioned", {"ok": True})
    rows = store.training_rows()
    assert rows[0]["expected"] == {"real_drop": True, "recoverable": True} and rows[0]["questions"] is QUESTIONS
    ag = store.agreement()
    assert ag["verdicts"] == 1 and ag["rules_accuracy"] == 1.0 and ag["laya_accuracy"] == 0.0
    assert store.mark_cleared(3) == [a["id"]]


def test_numbers_become_the_same_sentence_for_inference_and_training():
    from overwatch.decision.describe import describe
    real = drop(area=0.07, solidity=0.95, aspect=1.2, edge_density=0.2, contrast=95.0, stationary_s=4.0)
    s = describe(real.kind, real.features)
    assert "tiny object" in s and "compact and solid" in s and "sharp, well defined edges" in s
    smudge = describe(DROPPED_ITEM, {"area_pct": 0.3, "solidity": 0.6, "aspect": 3.0, "edge_density": 0.0,
                                     "contrast": 30.0, "stationary_s": 3.0})
    assert "irregular and elongated" in smudge and "soft, blurry edges" in smudge and "faintly different" in smudge
    assert real.to_state() == {"report": s}
