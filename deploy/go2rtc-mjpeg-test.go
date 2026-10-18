package mjpeg

import (
 "context"
 "encoding/json"
 "io"
 "net/http"
 "net/http/httptest"
 "sync"
 "testing"
 "time"

 "github.com/AlexxIT/go2rtc/internal/streams"
 "github.com/AlexxIT/go2rtc/pkg/core"
 pkgmjpeg "github.com/AlexxIT/go2rtc/pkg/mjpeg"
 "github.com/pion/rtp"
)

// An in-memory source negotiates JPEG and then sends nothing. No HTTP listener.
type lifecycleSilentProducer struct {
 core.Connection
 started, stopped chan struct{}
 once sync.Once
}
func (p *lifecycleSilentProducer) Start() error { close(p.started); <-p.stopped; return nil }
func (p *lifecycleSilentProducer) Stop() error { p.once.Do(func(){ _ = p.Connection.Stop(); close(p.stopped) }); return nil }

type lifecycleFailedWriter struct { header http.Header }
func (w *lifecycleFailedWriter) Header() http.Header { return w.header }
func (w *lifecycleFailedWriter) WriteHeader(int) {}
func (w *lifecycleFailedWriter) Write([]byte) (int,error) { return 0,io.ErrClosedPipe }

// Close is the only cancellation mechanism for this in-memory transport.
type lifecycleBlockingCloser struct {
 entered, release chan struct{}
 once sync.Once
}
func (w *lifecycleBlockingCloser) Write([]byte) (int,error) {
 close(w.entered)
 <-w.release
 return 0,io.ErrClosedPipe
}
func (w *lifecycleBlockingCloser) Close() error { w.once.Do(func(){ close(w.release) }); return nil }

func TestLifecycleWriteBufferCloseCancelsBlockedTransport(t *testing.T) {
 for _, mode := range []string{"write", "reset"} { t.Run(mode,func(t *testing.T) {
  wr := &lifecycleBlockingCloser{entered: make(chan struct{}),release: make(chan struct{})}
  defer wr.Close()
  b := core.NewWriteBuffer(nil)
  active := make(chan struct{})
  if mode=="write" {
   b.Reset(wr)
   go func(){ _,_ = b.Write([]byte("frame")); close(active) }()
  } else {
   _,_ = b.Write([]byte("buffered frame"))
   go func(){ b.Reset(wr); close(active) }()
  }
  select { case <-wr.entered: case <-time.After(time.Second): t.Fatal("transport never entered Write") }
  closed := make(chan struct{})
  go func(){ _ = b.Close(); close(closed) }()
  select {
  case <-closed:
  case <-time.After(250*time.Millisecond):
   _ = wr.Close(); <-active; <-closed
   t.Fatal("WriteBuffer.Close could not cancel the blocked transport")
  }
  select { case <-active: case <-time.After(time.Second): t.Fatal("transport Write was not joined") }
 }) }
}

type lifecycleCountingWriter struct { writes, flushes int; written chan struct{} }
func (w *lifecycleCountingWriter) Write(p []byte) (int,error) { w.writes++; if w.written!=nil { close(w.written) }; return len(p),nil }
func (w *lifecycleCountingWriter) Flush() { w.flushes++ }

func TestLifecycleWriteBufferTerminal(t *testing.T) {
 t.Run("reject writes",func(t *testing.T) {
  wr := &lifecycleCountingWriter{}
  b := core.NewWriteBuffer(wr)
  _ = b.Close()
  if n,err := b.Write([]byte("late frame")); n!=0 || err!=io.ErrClosedPipe { t.Errorf("closed Write = %d,%v; want 0,ErrClosedPipe",n,err) }
  if wr.writes!=0 || wr.flushes!=0 { t.Errorf("closed Write touched writer: %+v",wr) }
 })
 t.Run("discard buffered reset",func(t *testing.T) {
  b := core.NewWriteBuffer(nil)
  _,_ = b.Write([]byte("buffered frame"))
  _ = b.Close()
  wr := &lifecycleCountingWriter{}
  b.Reset(wr)
  if wr.writes!=0 || wr.flushes!=0 { t.Errorf("closed Reset flushed writer: %+v",wr) }
  if _,err := b.WriteTo(wr); err!=io.ErrClosedPipe { t.Errorf("closed WriteTo = %v; want ErrClosedPipe",err) }
 })
}

// A real MJPEG writer: its first frame is in flight during cancellation, and
// queued Sender callbacks are released only after the HTTP handler has returned.
type lifecycleResponseWriter struct {
 header http.Header
 mu sync.Mutex
 entered, release chan struct{}
 once sync.Once
 returned bool
 writes, flushes, late int
}
func (w *lifecycleResponseWriter) Header() http.Header { return w.header }
func (w *lifecycleResponseWriter) WriteHeader(int) {}
func (w *lifecycleResponseWriter) Write(p []byte) (int,error) {
 w.once.Do(func(){ close(w.entered); <-w.release })
 w.mu.Lock(); defer w.mu.Unlock()
 w.writes++; if w.returned { w.late++ }
 return len(p),nil
}
func (w *lifecycleResponseWriter) Flush() { w.mu.Lock(); defer w.mu.Unlock(); w.flushes++; if w.returned { w.late++ } }
type lifecycleObservedConsumer struct {
 *pkgmjpeg.Consumer
 stopping chan struct{}
 once sync.Once
}
func (c *lifecycleObservedConsumer) Stop() error { c.once.Do(func(){ close(c.stopping) }); return c.Consumer.Stop() }

func TestLifecycleMJPEGQueuedCancellation(t *testing.T) {
 media := &core.Media{Kind: core.KindVideo, Direction: core.DirectionRecvonly, Codecs: []*core.Codec{{Name: core.CodecJPEG,PayloadType: core.PayloadTypeRAW}}}
 p := &lifecycleSilentProducer{Connection: core.Connection{Medias: []*core.Media{media}},started: make(chan struct{}),stopped: make(chan struct{})}
 stream := streams.NewStream(nil)
 stream.AddProducer(p)
 cons := &lifecycleObservedConsumer{Consumer: pkgmjpeg.NewConsumer(),stopping: make(chan struct{})}
 if err := stream.AddConsumer(cons); err!=nil { t.Fatal(err) }
 // A second consumer on the same producer must survive the canceled consumer.
 other := pkgmjpeg.NewConsumer()
 if err := stream.AddConsumer(other); err!=nil { t.Fatal(err) }
 defer stream.RemoveConsumer(other)
 otherWriter := &lifecycleCountingWriter{written: make(chan struct{})}
 other.Transport.(*core.WriteBuffer).Reset(otherWriter)
 sender := cons.Senders[0]
 nextEntered, nextRelease := make(chan struct{}),make(chan struct{})
 original := sender.Output
 calls := 0
 sender.Output = func(packet *core.Packet) {
  calls++
  if calls==2 { close(nextEntered); <-nextRelease }
  original(packet)
 }
 wr := &lifecycleResponseWriter{header: make(http.Header),entered: make(chan struct{}),release: make(chan struct{})}
 ctx,cancel := context.WithCancel(context.Background()); defer cancel()
 cleanup := cancelHTTPConsumer(ctx,stream,cons)
 done := make(chan struct{})
 go func(){
  _,_ = cons.WriteTo(pkgmjpeg.NewWriter(wr))
  cleanup()
  wr.mu.Lock(); wr.returned=true; wr.mu.Unlock()
  close(done)
 }()
 packet := &rtp.Packet{Payload: []byte{0xff,0xd8,0xff,0xd9}}
 sender.Input(packet)
 select { case <-wr.entered: case <-time.After(time.Second): t.Fatal("first HTTP write not in flight") }
 sender.Input(packet); sender.Input(packet)
 cancel()
 select { case <-cons.stopping: case <-time.After(time.Second): t.Fatal("cancel callback did not start") }
 select { case <-done: t.Error("handler returned while HTTP write was in flight"); default: }
 close(wr.release)
 select { case <-nextEntered: case <-time.After(time.Second): t.Fatal("queued callback did not enter") }
 select { case <-done: case <-time.After(time.Second): t.Fatal("handler did not complete cancellation") }
 close(nextRelease)
 sender.Wait()
 wr.mu.Lock()
 writes,flushes,late := wr.writes,wr.flushes,wr.late
 wr.mu.Unlock()
 if writes!=1 || flushes!=1 || late!=0 { t.Errorf("HTTP writes=%d flushes=%d late=%d; want 1,1,0",writes,flushes,late) }
 // Send directly to the surviving sender to avoid racing the upstream Receiver
 // graph with producer cleanup; Sender.Input is the real packet queue API.
 other.Senders[0].Input(packet)
 select { case <-otherWriter.written: case <-time.After(time.Second): t.Fatal("surviving consumer did not receive frame") }
 stream.RemoveConsumer(other)
 other.Senders[0].Wait()
 if otherWriter.writes!=1 { t.Errorf("other consumer writes=%d; want 1",otherWriter.writes) }
 select { case <-p.stopped: t.Error("external producer was unexpectedly stopped"); default: }
}

func TestLifecycleMJPEGCloseBeforeWriteTo(t *testing.T) {
 // Cancellation can run between AddConsumer and the handler's first WriteTo.
 b := core.NewWriteBuffer(nil)
 _ = b.Close()
 done := make(chan struct{})
 go func(){ _,_ = b.WriteTo(io.Discard); close(done) }()
 select {
 case <-done:
 case <-time.After(250*time.Millisecond):
  _ = b.Close(); <-done
  t.Fatal("consumer canceled before WriteTo remained blocked")
 }
}

func TestLifecycleMJPEGCancellationAfterSilence(t *testing.T) {
 for _, endpoint := range []string{"stream.mjpeg", "frame.jpeg"} { t.Run(endpoint,func(t *testing.T) {
  p := &lifecycleSilentProducer{
   Connection: core.Connection{Medias: []*core.Media{{Kind: core.KindVideo, Direction: core.DirectionRecvonly, Codecs: []*core.Codec{{Name: core.CodecJPEG, PayloadType: core.PayloadTypeRAW}}}}},
   started: make(chan struct{}), stopped: make(chan struct{}),
  }
  streams.HandleFunc("lifecycle-silent",func(string)(core.Producer,error){ return p,nil })
  s,err := streams.New("lifecycle-cancel", "lifecycle-silent:fixture"); if err != nil { t.Fatal(err) }
  defer streams.Delete("lifecycle-cancel")
  ctx,cancel := context.WithCancel(context.Background()); defer cancel()
  r := httptest.NewRequest("GET", "/api/"+endpoint+"?src=lifecycle-cancel",nil).WithContext(ctx)
  w := &lifecycleFailedWriter{header: make(http.Header)}
  done := make(chan struct{})
  go func(){ if endpoint=="stream.mjpeg" { outputMjpeg(w,r) } else { handlerKeyframe(w,r) }; close(done) }()
  select { case <-p.started: case <-time.After(time.Second): t.Fatal("consumer was not added") }
  // Producer startup happens only after consumer registration. Deliberately no RTP.
  time.Sleep(100*time.Millisecond)
  select { case <-done: t.Fatal("handler did not wait for silent media"); default: }
  cancel()
  select {
  case <-done:
  case <-time.After(750*time.Millisecond):
   // Unwind the baseline too: a packet triggers the existing WriteTo error path.
   p.Receivers[0].WriteRTP(&rtp.Packet{Payload: []byte{0xff,0xd8,0xff,0xd9}})
   select { case <-done: case <-time.After(time.Second): t.Fatal("fixture cleanup failed") }
   t.Fatal("canceled HTTP consumer stayed blocked waiting for media")
  }
  select { case <-p.stopped: case <-time.After(time.Second): t.Fatal("last canceled consumer did not release producer") }
  b,err := s.MarshalJSON(); if err != nil { t.Fatal(err) }
  var info struct { Consumers []json.RawMessage `json:"consumers"` }
  if err := json.Unmarshal(b,&info); err != nil { t.Fatal(err) }
  if len(info.Consumers)!=0 { t.Fatalf("canceled consumer remained registered: %s",b) }
 }) }
}
