package exec

import (
 "fmt"
 "os"
 "path/filepath"
 "strconv"
 "strings"
 "syscall"
 "testing"
 "time"

 "github.com/AlexxIT/go2rtc/internal/streams"
 "github.com/AlexxIT/go2rtc/pkg/core"
 "github.com/AlexxIT/go2rtc/pkg/mjpeg"
)

// Only fixture subprocesses run here: no sockets, media devices or robot credentials.
func TestLifecyclePipeHelper(t *testing.T) {
 if len(os.Args) < 5 || os.Args[2] != "pipe-fixture" { return }
 mode, pidFile := os.Args[3], os.Args[4]
 if err := os.WriteFile(pidFile, []byte(strconv.Itoa(os.Getpid())), 0600); err != nil { os.Exit(2) }
 switch mode {
 case "good": fmt.Print("YUV4MPEG2 W2 H2 F1:1 Ip A1:1 C420jpeg\n")
 case "partial": fmt.Print("YUV4")
 case "exit": os.Exit(3)
 }
 time.Sleep(15*time.Second)
 os.Exit(0)
}

func fixtureURL(t *testing.T, mode string) (string, string) {
 t.Helper()
 bin, err := os.Executable(); if err != nil { t.Fatal(err) }
 pid := filepath.Join(t.TempDir(), "pid")
 return fmt.Sprintf("exec:%s -test.run=^TestLifecyclePipeHelper$ pipe-fixture %s %s#starttimeout=1", bin, mode, pid), pid
}

func fixturePID(t *testing.T, path string) int {
 t.Helper()
 end := time.Now().Add(time.Second)
 for time.Now().Before(end) {
  if b, err := os.ReadFile(path); err == nil { n, err := strconv.Atoi(string(b)); if err != nil { t.Fatal(err) }; return n }
  time.Sleep(10*time.Millisecond)
 }
 t.Fatal("fixture child did not start"); return 0
}

func assertReaped(t *testing.T, pid int) {
 t.Helper()
 // ESRCH, rather than just a signal ACK, proves no live or zombie child remains.
 if err := syscall.Kill(pid, 0); err != syscall.ESRCH { t.Fatalf("fixture child %d not reaped: %v", pid, err) }
}

func TestLifecyclePipeStartupTimeout(t *testing.T) {
 for _, mode := range []string{"silent", "partial"} { t.Run(mode, func(t *testing.T) {
  raw, pidFile := fixtureURL(t, mode)
  done := make(chan error, 1)
  go func() { p, err := execHandle(raw); if p != nil { _ = p.Stop() }; done <- err }()
  pid := fixturePID(t, pidFile)
  defer syscall.Kill(pid, syscall.SIGKILL)
  select {
  case err := <-done:
   if err == nil || !strings.Contains(err.Error(), "timeout") { t.Fatalf("expected startup timeout, got %v", err) }
   assertReaped(t, pid)
  case <-time.After(2500*time.Millisecond):
   _ = syscall.Kill(pid, syscall.SIGKILL); <-done
   t.Fatal("magic.Open stayed blocked beyond configured starttimeout")
  }
 }) }
}

func TestLifecyclePipePendingRecoveryAndReopen(t *testing.T) {
 streams.HandleFunc("exec", execHandle)
 raw, pidFile := fixtureURL(t, "silent")
 s := streams.NewStream(raw)
 done := make(chan error, 1)
 c := mjpeg.NewConsumer()
 go func() { done <- s.AddConsumer(c) }()
 pid := fixturePID(t, pidFile)
 defer syscall.Kill(pid, syscall.SIGKILL)
 select {
 case err := <-done:
  if err == nil { t.Fatal("silent source unexpectedly added") }
  assertReaped(t, pid)
 case <-time.After(2500*time.Millisecond):
  _ = syscall.Kill(pid, syscall.SIGKILL); <-done
  t.Fatal("pending AddConsumer held producer startup lock")
 }
 // Explicit subsequent demand, not a retry/reconnect loop: the same stream must recover.
 good, _ := fixtureURL(t, "good")
 s.SetSource(good)
 for i:=0; i<2; i++ {
  c = mjpeg.NewConsumer()
  added := make(chan error, 1)
  go func() { added <- s.AddConsumer(c) }()
  select { case err := <-added: if err != nil { t.Fatal(err) }; case <-time.After(2*time.Second): t.Fatal("successful source could not reopen after failure/idle") }
  removed := make(chan struct{})
  go func() { s.RemoveConsumer(c); close(removed) }()
  select { case <-removed: case <-time.After(2*time.Second): t.Fatal("producer cleanup lock remained held") }
 }
}

func TestLifecyclePipeSuccessStopsStartupTimer(t *testing.T) {
 raw, pidFile := fixtureURL(t, "good")
 p, err := execHandle(raw); if err != nil { t.Fatal(err) }
 defer p.Stop()
 pid := fixturePID(t, pidFile)
 defer syscall.Kill(pid, syscall.SIGKILL)
 if p.GetMedias()[0].Codecs[0].Name != core.CodecRAW { t.Fatal("fixture media did not open") }
 time.Sleep(1250*time.Millisecond)
 if err := syscall.Kill(pid, 0); err != nil { t.Fatalf("startup timer killed an already-open source: %v", err) }
}

func TestLifecyclePipeEarlyFailureIsBounded(t *testing.T) {
 raw, pidFile := fixtureURL(t, "exit")
 done := make(chan error, 1)
 go func(){ _, err := execHandle(raw); done <- err }()
 pid := fixturePID(t, pidFile)
 select { case err := <-done: if err == nil { t.Fatal("expected early child failure") }; case <-time.After(2*time.Second): t.Fatal("early failure did not unwind") }
 // Normal failure is asynchronously reaped by shell.Command too.
 end := time.Now().Add(time.Second)
 for syscall.Kill(pid,0) != syscall.ESRCH && time.Now().Before(end) { time.Sleep(10*time.Millisecond) }
 assertReaped(t,pid)
}
