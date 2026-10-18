# Shared HTTP/MCP contract — current owner reference

The living route/field/error contract is [thin-api.md](../thin-api.md).
[The ROS guide](../ros2.md) owns ROS endpoint semantics. This path is retained for
existing links, not as a second normative phase-specific API.

Source authority:
- [Gateway routes, goal validation, task registry and frame matching](../../src/z70_web_gateway/z70_web_gateway/gateway.py).
- [MCP thin client](../../apps/mcp/src/mcp-server.ts).
- [Private authenticated facade](../../adapters/vendor/src/service.ts).
- [Generated actions/services/messages](../../src/z70_interfaces/CMakeLists.txt).

The extracted contract preserves real ROS goal UUIDs as lowercase hyphenated task
handles, generated snake_case feedback/results, bounded task history and explicit
unknown outcomes on lost replies. A gateway restart loses its in-memory history;
it never invents completion or replays writes. Frame requests filter the shared
ROS cache (maximum five-second receipt age), never acquire independently or switch
and restore a lens. Selection is a separate shared operation. Control-session
stop/resume uses `/api/ros/control/session`; state includes `session_enabled`.

The initial phase-4 five-kind list is obsolete: current source has eleven action
kinds, including Stack and the curated arm/alignment stages. Authentication/setup
is outside DDS; first-run setup availability belongs to current onboarding and
source, not the old configured-browser PIN flow. The current guide distinguishes
setup availability, ROS scheduling, transport
ACK and physical completion rather than treating HTTP acceptance as robot success.

Historical shared-MCP/UI/DDS test scope remains in [phase 4](phase4.md). No deployment
or live authorization transfers from that record to a new session.
