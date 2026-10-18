package streams

import (
 "errors"
 "sync"
 "sync/atomic"
 "testing"
 "time"

 "github.com/AlexxIT/go2rtc/pkg/core"
)

// Only in-memory fixtures: registering the real scheme exercises GetProducer,
// Stream.AddConsumer, worker exit and reconnect without MQTT/HTTP or a robot.
type lifecycleExitProducer struct {
 core.Connection
 started, exit, stopped chan struct{}
 result error
 once sync.Once
}
func lifecycleProducer(result error) *lifecycleExitProducer {
 return &lifecycleExitProducer{Connection: core.Connection{Medias: []*core.Media{{Kind: core.KindVideo, Direction: core.DirectionRecvonly, Codecs: []*core.Codec{{Name: core.CodecJPEG, PayloadType: core.PayloadTypeRAW}}}}}, started: make(chan struct{}), exit: make(chan struct{}), stopped: make(chan struct{}), result: result}
}
func (p *lifecycleExitProducer) Start() error { close(p.started); select { case <-p.exit: case <-p.stopped: }; return p.result }
func (p *lifecycleExitProducer) Stop() error { p.once.Do(func(){ _ = p.Connection.Stop(); close(p.stopped) }); return nil }

type lifecycleTrackConsumer struct { core.Connection }
func lifecycleConsumer() *lifecycleTrackConsumer {
 return &lifecycleTrackConsumer{core.Connection{Medias: []*core.Media{{Kind: core.KindVideo, Direction: core.DirectionSendonly, Codecs: []*core.Codec{{Name: core.CodecJPEG, PayloadType: core.PayloadTypeRAW}}}}}}
}
func (c *lifecycleTrackConsumer) AddTrack(media *core.Media, codec *core.Codec, track *core.Receiver) error {
 sender := core.NewSender(media,codec)
 sender.Handler = func(*core.Packet){}
 sender.HandleRTP(track)
 c.Senders = append(c.Senders,sender)
 return nil
}
func lifecycleAwait(t *testing.T, ch <-chan struct{}, what string) {
 t.Helper(); select { case <-ch: case <-time.After(time.Second): t.Fatalf("timed out: %s",what) }
}
func lifecycleRegister(t *testing.T, scheme string, h Handler) {
 t.Helper(); old,exists := handlers[scheme]; HandleFunc(scheme,h)
 t.Cleanup(func(){ if exists { handlers[scheme]=old } else { delete(handlers,scheme) } })
}

func TestLifecycleRoborockNoAutomaticReopen(t *testing.T) {
 for _, result := range []error{nil,errors.New("fixture transport failed")} { name := "normal-exit"; if result!=nil { name="transport-error" }; t.Run(name,func(t *testing.T){
  first,second := lifecycleProducer(result),lifecycleProducer(result)
  var dials atomic.Int32
  reopened := make(chan struct{},1)
  var explicit atomic.Bool
  lifecycleRegister(t,"roborock",func(string)(core.Producer,error){
   n := dials.Add(1)
   if n==1 { return first,nil }
   if n==2 && explicit.Load() { return second,nil }
   select { case reopened<-struct{}{}: default: }
   return nil,errors.New("unexpected automatic preview reopen")
  })
  s:=NewStream("roborock:offline-fixture")
  c:=lifecycleConsumer()
  t.Cleanup(func(){ s.RemoveConsumer(c); _=first.Stop(); _=second.Stop() })
  if err:=s.AddConsumer(c); err!=nil { t.Fatal(err) }
  lifecycleAwait(t,first.started,"first Start")
  track:=first.Receivers[0]
  close(first.exit)
  select {
  case <-first.stopped:
  case <-reopened: t.Fatalf("automatic preview-producing reconnect: Dial count=%d, want 1",dials.Load())
  case <-time.After(time.Second): t.Fatal("exited camera connection was not stopped")
  }
  p:=s.producers[0]
  // Stop signals while reconnect still holds the mutex; this is a completion barrier.
  p.mu.Lock(); idle:=p.state==stateNone && p.conn==nil && len(p.receivers)==0 && len(p.senders)==0; p.mu.Unlock()
  if !idle { t.Fatal("failed producer retained connection/track state") }
  if len(track.Senders())!=0 { t.Fatal("failed producer retained consumer attachment") }
  // Caller cleanup removes the registration and closes consumer-owned buffers.
  s.RemoveConsumer(c)
  s.mu.Lock(); remaining:=len(s.consumers); s.mu.Unlock()
  if remaining!=0 { t.Fatal("consumer registration not released after cleanup") }
  time.Sleep(1100*time.Millisecond) // spans upstream's first background retry
  if n:=dials.Load(); n!=1 { t.Fatalf("background Dial count=%d, want 1",n) }
  explicit.Store(true)
  next:=lifecycleConsumer(); defer s.RemoveConsumer(next)
  if err:=s.AddConsumer(next); err!=nil { t.Fatal(err) }
  lifecycleAwait(t,second.started,"explicit next demand")
  if n:=dials.Load(); n!=2 { t.Fatalf("explicit next demand Dial count=%d, want 2",n) }
 }) }
}

func TestLifecycleOtherProducerStillReconnects(t *testing.T) {
 // Similar prefix must not match the exact Roborock scheme.
 first,second:=lifecycleProducer(nil),lifecycleProducer(nil)
 var dials atomic.Int32
 lifecycleRegister(t,"roborock-fixture",func(string)(core.Producer,error){ if dials.Add(1)==1 { return first,nil }; return second,nil })
 s:=NewStream("roborock-fixture:offline")
 c:=lifecycleConsumer(); defer s.RemoveConsumer(c)
 if err:=s.AddConsumer(c); err!=nil { t.Fatal(err) }
 lifecycleAwait(t,first.started,"first Start")
 old:=first.Receivers[0]
 close(first.exit)
 lifecycleAwait(t,second.started,"ordinary reconnect Start")
 lifecycleAwait(t,first.stopped,"old connection stopped")
 p:=s.producers[0]; p.mu.Lock(); current:=p.conn==second && p.state==stateStart; p.mu.Unlock()
 if !current || dials.Load()!=2 { t.Fatal("ordinary producer did not reconnect normally") }
 if len(old.Senders())!=0 || len(second.Receivers[0].Senders())!=1 { t.Fatal("ordinary reconnect did not transfer consumer") }
}

func TestLifecycleRoborockStaleWorkerPreservesNewGeneration(t *testing.T) {
 first,second:=lifecycleProducer(nil),lifecycleProducer(nil)
 var dials atomic.Int32
 lifecycleRegister(t,"roborock",func(string)(core.Producer,error){ if dials.Add(1)==1 { return first,nil }; return second,nil })
 s:=NewStream("roborock:offline-stale")
 c:=lifecycleConsumer()
 if err:=s.AddConsumer(c); err!=nil { t.Fatal(err) }
 lifecycleAwait(t,first.started,"old Start")
 p:=s.producers[0]; p.mu.Lock(); stale:=p.workerID; p.mu.Unlock()
 s.RemoveConsumer(c)
 next:=lifecycleConsumer(); defer s.RemoveConsumer(next)
 if err:=s.AddConsumer(next); err!=nil { t.Fatal(err) }
 lifecycleAwait(t,second.started,"new Start")
 // Exercise the exact stale-worker completion seam deterministically.
 p.reconnect(stale,0)
 p.mu.Lock(); intact:=p.conn==second && p.state==stateStart; p.mu.Unlock()
 select { case <-second.stopped: t.Fatal("stale worker stopped newer connection"); default: }
 if !intact || dials.Load()!=2 || len(second.Receivers[0].Senders())!=1 { t.Fatal("stale worker changed newer generation") }
}
