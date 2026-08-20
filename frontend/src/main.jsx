import { createRoot } from 'react-dom/client'
import './index.css'
import App from './App.jsx'

// No <StrictMode> -- its dev-only intentional double-mount (mount->cleanup
// ->mount, to help surface effect bugs) was the root cause of the Google
// Maps markers going invisible on every real page refresh (see
// RobotMap.jsx and README.md "Map" section for the full story). The
// effects there are now written to survive it, but StrictMode is dev-only
// tooling with zero effect on production behavior, so removing it trades
// a class of double-mount footguns for slightly less automated bug-
// surfacing -- worth it here given how much time this cost.
createRoot(document.getElementById('root')).render(<App />)
