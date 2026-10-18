import test from "node:test";
import assert from "node:assert/strict";
import { manualJointFixture, parseProgramArmFixture, ARM_INTERFACE_RESEARCH } from "./legacy-ts/arm-joint-protocol-research.ts";
import { MANUAL_JOINT_CONTROLS } from "../adapters/vendor/src/arm-joint-protocol.ts";

test("Hermes manual button fixtures use exact three-entry selections, never angle targets", () => {
  const expected = [[2, 0, 0], [1, 0, 0], [0, 1, 0], [0, 2, 0], [0, 0, 2], [0, 0, 1]];
  Object.keys(MANUAL_JOINT_CONTROLS).forEach((control, index) => {
    const fixture = manualJointFixture(control as keyof typeof MANUAL_JOINT_CONTROLS);
    assert.deepEqual(fixture.params, { m: expected[index] });
    assert.equal(fixture.executionAvailable, false);
    fixture.params.m[0] = 99;
    assert.deepEqual(manualJointFixture(control as keyof typeof MANUAL_JOINT_CONTROLS).params.m, expected[index]);
  });
  assert.throws(() => manualJointFixture("yaw-radians" as never));
});
test("program arm_joints vectors and arm_move directions cannot be interchanged", () => {
  const joints = { cmd_id: "arm_joints", value: [0, 0, 1, 0], step: 1 };
  assert.deepEqual(parseProgramArmFixture(joints), joints);
  assert.deepEqual(parseProgramArmFixture({ cmd_id: "arm_move", direction: 3, step: 1 }), { cmd_id: "arm_move", direction: 3, step: 1 });
  for (const invalid of [
    { cmd_id: "arm_move", value: [1, 0, 0, 0], step: 1 }, { cmd_id: "arm_joints", direction: 1, step: 1 },
    { ...joints, step: 2 }, { ...joints, value: [1, 2, 0, 0] }, { ...joints, value: [90, 0, 0, 0] },
    { ...joints, angle: 10 }, { ...joints, value: [true, 0, 0, 0] },
  ]) assert.throws(() => parseProgramArmFixture(invalid));
});
test("model degree increments and five-input wrapper do not advertise measured execution", () => {
  assert.equal(ARM_INTERFACE_RESEARCH.programModel.measured, false);
  assert.equal(ARM_INTERFACE_RESEARCH.fiveInputWrapper.absoluteTargetsVerified, false);
  assert.equal(ARM_INTERFACE_RESEARCH.jointQuery.measuredFeedbackVerified, false);
  assert.equal(ARM_INTERFACE_RESEARCH.cameraSelector.settingsBitFallbackEquivalent, false);
});
