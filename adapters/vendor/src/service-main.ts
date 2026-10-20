import { loadServiceConfig, startService } from "./service.ts";
import { SecretStore, safeReason } from "./secret-store.ts";


try {
  const config = loadServiceConfig();
  const service = await startService({ config, store: new SecretStore(config.stateDir) });
  let stopping = false;
  const shutdown = () => {
    if (stopping) return;
    stopping = true;
    void service.stop().then(() => {
      process.off("SIGTERM", shutdown); process.off("SIGINT", shutdown);
    }, () => {
      console.error("RoboClaw shutdown failed.");
      process.exitCode = 1;
    });
  };
  process.on("SIGTERM", shutdown);
  process.on("SIGINT", shutdown);
  console.log(`RoboClaw HTTP service listening on port ${config.port}.`);
} catch (error) {
  // Only fixed messages and safe reasons (errno codes, internal labels) are printed; never paths or secrets.
  const message = error instanceof Error && /^(Cannot|Invalid|Unsafe|Service)[ A-Za-z.]{0,80}$/.test(error.message) ? error.message : "";
  const cause = error instanceof Error && typeof error.cause === "string" ? error.cause : safeReason(error);
  console.error(`RoboClaw service could not start: ${message || "startup failed"} (${cause}).`);
  process.exitCode = 1;
}
