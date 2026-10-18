// Apply one fail-closed patch to the pinned go2rtc source, not a moving upstream checkout.
package main

import (
 "fmt"
 "os"
 "strings"
)

func main() {
 path := "pkg/roborock/iot/client.go"
 data, err := os.ReadFile(path); if err != nil { panic(err) }
 source := string(data)
 anchor := `c.mqtt.Connect("com.roborock.smart:mbrriot", user, pass)`
 if strings.Count(source, anchor) != 1 { panic("pinned MQTT callsite changed") }
 source = strings.Replace(source, anchor, `c.mqtt.Connect(cameraMQTTClientID(), user, pass)`, 1)
 helper := `func cameraMQTTClientID() string { return "com.roborock.smart:mbrriot" }`
 if len(os.Args) < 2 || os.Args[1] != "baseline" {
  importAnchor := `"github.com/AlexxIT/go2rtc/pkg/mqtt"`
  if strings.Count(source, importAnchor) != 1 { panic("pinned import changed") }
  source = strings.Replace(source, importAnchor, importAnchor+"\n\t\"github.com/google/uuid\"", 1)
  helper = `func cameraMQTTClientID() string { return "roboclaw-" + uuid.NewString() }`
 }
 source += "\n// Unique broker identity per camera connection; the legacy shared identity is rejected.\n"+helper+"\n"
 if err = os.WriteFile(path, []byte(source), 0644); err != nil { panic(err) }
 fmt.Println("Applied camera MQTT client-ID patch")
 if len(os.Args) < 2 || os.Args[1] != "baseline" { patchPipe(); patchMJPEG(); patchRoborockReconnect() }
}

// Exact anchors fail closed if the pinned upstream source ever changes.
func replaceFile(path, old, replacement string) {
 data, err := os.ReadFile(path); if err != nil { panic(err) }
 source := string(data)
 if strings.Count(source, old) != 1 { panic("pinned lifecycle anchor changed: " + path) }
 if err := os.WriteFile(path, []byte(strings.Replace(source, old, replacement, 1)), 0644); err != nil { panic(err) }
}

func patchRoborockReconnect() {
 // Inside the existing mutex, after stale-worker rejection and before Dial or
 // retry scheduling. Roborock Connect starts remote preview, so it is not an
 // innocuous transport retry. A fresh AddConsumer may Dial again after cleanup.
 replaceFile("internal/streams/producer.go", "\tlog.Debug().Msgf(\"[streams] retry=%d to url=%s\", retry, p.url)", `	if strings.HasPrefix(p.url, "roborock:") {
		p.workerID++
		if p.conn != nil {
			_ = p.conn.Stop()
			p.conn = nil
		}
		p.state = stateNone
		p.receivers = nil
		p.senders = nil
		return
	}

	log.Debug().Msgf("[streams] retry=%d to url=%s", retry, p.url)`)
 fmt.Println("Applied Roborock-only no-automatic-preview-reopen patch")
}

func patchPipe() {
 path := "internal/exec/exec.go"
 replaceFile(path, "handlePipe(rawURL, cmd)", "handlePipe(rawURL, cmd, timeout)")
 replaceFile(path, "func handlePipe(source string, cmd *shell.Command)", "func handlePipe(source string, cmd *shell.Command, timeout time.Duration)")
 replaceFile(path, "\tif err = cmd.Start(); err != nil {", `	// Bound inherited stderr-copy waits when a startup child does not exit cleanly.
	if cmd.WaitDelay == 0 { cmd.WaitDelay = time.Second }
	if err = cmd.Start(); err != nil {`)
 replaceFile(path, "\tprod, err := magic.Open(rd)", `	type opened struct { prod core.Producer; err error }
	ready := make(chan opened, 1)
	go func() { prod, err := magic.Open(rd); ready <- opened{prod, err} }()
	timer := time.NewTimer(timeout)
	defer timer.Stop()
	var prod core.Producer
	select {
	case result := <-ready:
		prod, err = result.prod, result.err
	case <-timer.C:
		// A silent/partial header must release both magic.Open and pending Dial.
		// Startup failure is terminal: do not allow a custom graceful signal to
		// leave the helper alive, and wait for the actual Wait goroutine to reap.
		_ = cmd.Close()
		_ = cmd.Process.Kill()
		_ = stdout.Close()
		<-cmd.Reaped()
		result := <-ready
		if result.err == nil && result.prod != nil { _ = result.prod.Stop() }
		return nil, errors.New("exec/pipe: startup timeout")
	}`)
 // shell.Done is a cancellation signal, not proof that Cmd.Wait has finished.
 replaceFile("pkg/shell/command.go", "\terr    error\n", "\terr    error\n\treaped chan struct{}\n")
 replaceFile("pkg/shell/command.go", "return &Command{cmd, ctx, cancel, nil}", "return &Command{cmd, ctx, cancel, nil, make(chan struct{})}")
 replaceFile("pkg/shell/command.go", "\t\tc.err = c.Cmd.Wait()", "\t\tc.err = c.Cmd.Wait()\n\t\tclose(c.reaped)")
 replaceFile("pkg/shell/command.go", "func (c *Command) Done() <-chan struct{} {", "// Reaped closes only after the child and command I/O have been waited for.\nfunc (c *Command) Reaped() <-chan struct{} { return c.reaped }\n\nfunc (c *Command) Done() <-chan struct{} {")
 fmt.Println("Applied bounded exec-pipe startup lifecycle patch")
}

func patchMJPEG() {
 path := "internal/mjpeg/mjpeg.go"
 replaceFile(path, "import (\n", "import (\n\t\"context\"\n")
 replaceFile(path, "\tonce := &core.OnceBuffer{} // init and first frame", `	cleanup := cancelHTTPConsumer(r.Context(), stream, cons)
	defer cleanup()
	once := &core.OnceBuffer{} // init and first frame`)
 replaceFile(path, "\tb = once.Buffer()\n\n\tstream.RemoveConsumer(cons)", "\tb = once.Buffer()\n\n\tcleanup()\n\tif r.Context().Err() != nil { return }")
 // Anchor inside outputMjpeg only, not WS or Y4M.
 replaceFile(path, "\th := w.Header()\n\th.Set(\"Cache-Control\", \"no-cache\")", "\tcleanup := cancelHTTPConsumer(r.Context(), stream, cons)\n\tdefer cleanup()\n\n\th := w.Header()\n\th.Set(\"Cache-Control\", \"no-cache\")")
 replaceFile(path, "\tstream.RemoveConsumer(cons)\n}\n\nfunc inputMjpeg", "}\n\nfunc inputMjpeg")
 replaceFile(path, "func inputMjpeg(w http.ResponseWriter, r *http.Request) {", `// Install only after successful AddConsumer: cancellation removes exactly once.
// Join an active callback before returning, so teardown cannot outlive the handler.
func cancelHTTPConsumer(ctx context.Context, stream *streams.Stream, cons core.Consumer) func() {
	var removeOnce, cleanupOnce sync.Once
	remove := func() { removeOnce.Do(func() { stream.RemoveConsumer(cons) }) }
	done := make(chan struct{})
	stop := context.AfterFunc(ctx, func() { remove(); close(done) })
	return func() {
		cleanupOnce.Do(func() {
			if !stop() { <-done }
			remove()
		})
	}
}

func inputMjpeg(w http.ResponseWriter, r *http.Request) {`)
 // Cancellation may beat WriteTo.Reset. Preserve the closed state in that race.
 replaceFile("pkg/core/writebuffer.go", "\tif w.state == start {\n\t\tw.state = end\n\t\tw.wg.Done()\n\t}", "\tif w.state == start {\n\t\tw.wg.Done()\n\t}\n\tw.state = end")
 // Writer publication has its own short lock: Close must cancel a blocked
 // transport before waiting for the write/flush lock. Reset publishes the new
 // target before flushing buffered data, so that flush is cancellable too.
 replaceFile("pkg/core/writebuffer.go", "\tmu    sync.Mutex", "\tmu    sync.Mutex\n\twriterMu sync.Mutex\n\tclosed bool")
 replaceFile("pkg/core/writebuffer.go", "\tw.mu.Lock()\n\tif w.err != nil {", "\tw.mu.Lock()\n\tw.writerMu.Lock()\n\tclosed := w.closed\n\tw.writerMu.Unlock()\n\tif closed && w.err == nil { w.err = io.ErrClosedPipe; w.done() }\n\tif w.err != nil {")
 replaceFile("pkg/core/writebuffer.go", "\tw.add()\n\tif buf, ok := w.Writer.(*bytes.Buffer);", "\tw.writerMu.Lock()\n\tif w.closed {\n\t\tw.writerMu.Unlock()\n\t\tif w.err == nil { w.err = io.ErrClosedPipe }\n\t\tw.done()\n\t\tw.mu.Unlock()\n\t\treturn\n\t}\n\tw.add()\n\tprevious := w.Writer\n\tw.Writer = wr\n\tw.writerMu.Unlock()\n\tif buf, ok := previous.(*bytes.Buffer);")
 replaceFile("pkg/core/writebuffer.go", "\tw.Writer = wr\n\tw.mu.Unlock()", "\tw.mu.Unlock()")
 // A WaitGroup wakeup is not an exclusive read of the terminal error: queued
 // Sender callbacks can still run. Read under the same lock as Write/Reset.
 replaceFile("pkg/core/writebuffer.go", "\treturn 0, w.err // TODO: fix counter", "\tw.mu.Lock()\n\terr = w.err\n\tw.mu.Unlock()\n\treturn 0, err // TODO: fix counter")
 replaceFile("pkg/core/writebuffer.go", `func (w *WriteBuffer) Close() error {
	if closer, ok := w.Writer.(io.Closer); ok {
		return closer.Close()
	}
	w.mu.Lock()
	w.done()
	w.mu.Unlock()
	return nil
}`, `func (w *WriteBuffer) Close() error {
	// Never hold writerMu while acquiring mu: Write/Reset lock in reverse order.
	// Publish closure and snapshot together, then cancel without the write lock.
	w.writerMu.Lock()
	w.closed = true
	closer, ok := w.Writer.(io.Closer)
	w.writerMu.Unlock()
	var err error
	if ok { err = closer.Close() }
	// Join an in-flight Write, Flush or Reset before waking WriteTo/returning.
	w.mu.Lock()
	if w.err == nil { w.err = io.ErrClosedPipe }
	w.done()
	w.mu.Unlock()
	return err
}`)
 fmt.Println("Applied request-context MJPEG consumer lifecycle patch")
}
