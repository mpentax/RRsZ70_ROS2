package iot

import (
 "strings"
 "testing"
 "github.com/google/uuid"
)

func TestCameraMQTTClientIDIsUnique(t *testing.T) {
 first, second := cameraMQTTClientID(), cameraMQTTClientID()
 if first == second { t.Fatal("camera connections reuse a shared broker identity") }
 for _, id := range []string{first, second} {
  if !strings.HasPrefix(id, "roboclaw-") { t.Fatal("unexpected client identity namespace") }
  if _, err := uuid.Parse(strings.TrimPrefix(id, "roboclaw-")); err != nil { t.Fatal("invalid unique identity") }
 }
}
