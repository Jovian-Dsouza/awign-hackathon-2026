"""Aggregations over persisted records. Nothing here is invented: an empty DB gives empty charts."""

from __future__ import annotations

import json
from collections import Counter
from datetime import UTC, datetime, timedelta
from statistics import median

from app.repository import Repository, parse_iso


def _category(reason: str) -> str:
    if reason.startswith("NFC: no tag"):
        return "NFC tag not read"
    if reason.startswith("NFC"):
        return "Wrong item (NFC)"
    if reason.startswith("Load cell") and "nothing placed" in reason:
        return "Empty placement (weight)"
    if reason.startswith("Load cell"):
        return "Weight out of tolerance"
    if reason.startswith("Robot did not complete"):
        return "Pick / grasp failure"
    if reason.startswith("Robot placed at"):
        return "Wrong destination"
    if reason.startswith("No independent evidence"):
        return "No sensor evidence"
    return "Other"


def _bucket_size(span: timedelta) -> timedelta:
    if span <= timedelta(hours=2):
        return timedelta(minutes=5)
    if span <= timedelta(hours=12):
        return timedelta(minutes=30)
    if span <= timedelta(days=3):
        return timedelta(hours=2)
    return timedelta(days=1)


def compute(repo: Repository) -> dict:
    rows = repo.analytics_rows()
    orders, tasks, qcs = rows["orders"], rows["tasks"], rows["qc"]

    # ---- throughput over time (created vs dispatched) ----
    throughput: list[dict] = []
    bucket_minutes = None
    created = [parse_iso(o["created_at"]) for o in orders]
    done = [parse_iso(o["completed_at"]) for o in orders if o["status"] == "dispatched" and o["completed_at"]]
    if created:
        now = datetime.now(UTC)
        start = min(created)
        size = _bucket_size(now - start)
        bucket_minutes = int(size.total_seconds() // 60)
        epoch = datetime(2000, 1, 1, tzinfo=UTC)
        first = epoch + ((start - epoch) // size) * size
        n = min(int((now - first) / size) + 1, 48)
        first = max(first, epoch + ((now - epoch) // size) * size - size * (n - 1))
        for i in range(n):
            lo, hi = first + size * i, first + size * (i + 1)
            throughput.append({
                "t": lo.isoformat().replace("+00:00", "Z"),
                "created": sum(lo <= c < hi for c in created),
                "fulfilled": sum(lo <= d < hi for d in done),
            })

    # ---- task outcomes by type ----
    by_type: dict[str, Counter] = {}
    for t in tasks:
        by_type.setdefault(t["type"], Counter())[t["status"]] += 1
    order_of = ["navigate", "identify", "pick", "transport", "place", "verify", "dispatch"]
    task_outcomes = [
        {"type": k, "completed": by_type[k]["completed"], "failed": by_type[k]["failed"],
         "awaiting_review": by_type[k]["awaiting_review"], "skipped": by_type[k]["skipped"]}
        for k in order_of if k in by_type
    ]
    finished = [t for t in tasks if t["status"] in ("completed", "failed")]
    success_rate = (sum(t["status"] == "completed" for t in finished) / len(finished)) if finished else None

    # ---- fulfillment duration ----
    durations = []
    for o in orders:
        if o["status"] == "dispatched" and o["completed_at"]:
            durations.append({"order_id": o["id"], "completed_at": o["completed_at"],
                              "seconds": round((parse_iso(o["completed_at"]) - parse_iso(o["created_at"])).total_seconds(), 1)})
    durations.sort(key=lambda d: d["completed_at"])
    secs = [d["seconds"] for d in durations]

    # ---- QC ----
    qc_counts = Counter(q["status"] for q in qcs)
    reviews = Counter(q["review_decision"] for q in qcs if q["review_decision"])
    categories: Counter = Counter()
    for q in qcs:
        for reason in json.loads(q["reasons"]):
            categories[_category(reason)] += 1

    return {
        "data_source": "simulated" if any(t["source"] == "simulated" for t in tasks) else "recorded",
        "totals": {
            "orders": len(orders),
            "dispatched": len(done),
            "exceptions": sum(o["status"] == "exception" for o in orders),
            "tasks_finished": len(finished),
        },
        "throughput": {"bucket_minutes": bucket_minutes, "points": throughput},
        "task_outcomes": task_outcomes,
        "task_success_rate": round(success_rate, 4) if success_rate is not None else None,
        "fulfillment": {
            "recent": durations[-20:],
            "avg_seconds": round(sum(secs) / len(secs), 1) if secs else None,
            "median_seconds": round(median(secs), 1) if secs else None,
        },
        "qc": {
            "PASS": qc_counts["PASS"], "FAIL": qc_counts["FAIL"], "REVIEW": qc_counts["REVIEW"],
            "PENDING": qc_counts["PENDING"], "approved": reviews["approved"], "rejected": reviews["rejected"],
        },
        "exception_categories": [{"category": k, "count": v} for k, v in categories.most_common()],
    }
