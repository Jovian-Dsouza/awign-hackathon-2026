import { Link } from 'react-router-dom'

export default function Home() {
  return (
    <main className="home">
      <h1>Dark-Store Robot</h1>
      <p>Mecanum base · SO-101 arm · LeRobot ACT policy</p>
      <Link className="btn" to="/model">
        View 3D model
      </Link>
    </main>
  )
}
