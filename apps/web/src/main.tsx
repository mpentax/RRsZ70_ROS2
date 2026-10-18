import { createRoot } from "react-dom/client";
import { App } from "./App.tsx";
import { initialSession } from "./api.ts";
import "./styles.css";

// Keep the one-shot session check outside effect cleanup/replay; React receives only its shared result promise.
const session = initialSession();
const root = document.getElementById("root");
if (!root) throw new Error("RoboClaw root element is missing.");
createRoot(root).render(<App session={session} />);
