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
 if len(os.Args) < 2 || os.Args[1] != "baseline" { patchPipe(); patchMJPEG(); patchRoborockReconnect(); patchDiagnostics() }
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

 func writeFile(path, text string) {
 if err := os.WriteFile(path, []byte(text), 0644); err != nil { panic(err) }
 }

 func patchDiagnostics() {
 if err := os.MkdirAll("pkg/roborock/diagnostic", 0755); err != nil { panic(err) }
 writeFile("pkg/roborock/diagnostic/diagnostic.go", diagnosticSource)
 const client = "pkg/roborock/client.go"
 replaceFile(client, `"github.com/AlexxIT/go2rtc/pkg/roborock/iot"`, `"github.com/AlexxIT/go2rtc/pkg/roborock/iot"`+"\n\t\"github.com/AlexxIT/go2rtc/pkg/roborock/diagnostic\"")
 replaceFile(client, "\turl string", "\turl string\n\tdiagnostic *diagnostic.Attempt")
 replaceFile(client, "func (c *Client) Dial() error {", `func (c *Client) Dial() (err error) {
 c.diagnostic = diagnostic.Begin()
 defer func(){ if err != nil { c.diagnostic.Finish(err) } }()
 c.diagnostic.Stage("parse_url")`)
 replaceFile(client, "iot.Dial(c.url)", "iot.DialWithDiagnostic(c.url, c.diagnostic)")
 replaceFile(client, "\tc.pin = u.Query().Get(\"pin\")", "\tc.diagnostic.Stage(\"pin\")\n\tc.pin = u.Query().Get(\"pin\")")
 replaceFile(client, "\t\treturn c.CheckHomesecPassword()", "\t\terr = c.CheckHomesecPassword()\n\t\tc.diagnostic.Result(\"pin\", err)\n\t\treturn err")
 replaceFile(client, "\treturn nil\n}\n\nfunc (c *Client) Connect() error {", ` c.diagnostic.SkipPIN()
 return nil
 }

 func (c *Client) Connect() (err error) {
 defer func(){ c.diagnostic.Finish(err) }()`)
 replaceFile(client, "\t\tclientID, err := c.GetHomesecConnectStatus()", "\t\tc.diagnostic.Stage(\"status\")\n\t\tclientID, err := c.GetHomesecConnectStatus()")
 replaceFile(client, "\t\tif err = c.StopCameraPreview(clientID); err != nil {", "\t\tc.diagnostic.Stage(\"stop_preview\")\n\t\tif err = c.StopCameraPreview(clientID); err != nil {")
 replaceFile(client, "\tif err := c.StartCameraPreview(); err != nil {", "\tc.diagnostic.Stage(\"start_preview\")\n\tif err := c.StartCameraPreview(); err != nil {")
 replaceFile(client, "\tif turn, _ := c.GetTurnServer(); turn != nil {", "\tc.diagnostic.Stage(\"turn\")\n\tturn, turnErr := c.GetTurnServer()\n\tc.diagnostic.Result(\"turn\", turnErr)\n\tif turn != nil {")
 replaceFile(client, "\tapi, err := webrtc.NewAPI()", "\tc.diagnostic.Stage(\"create_api\")\n\tapi, err := webrtc.NewAPI()")
 replaceFile(client, "\tpc, err := api.NewPeerConnection(conf)", "\tc.diagnostic.Stage(\"create_peer\")\n\tpc, err := api.NewPeerConnection(conf)")
 replaceFile(client, "\tc.conn = webrtc.NewConn(pc)", "\tc.conn = webrtc.NewConnWithDiagnostic(pc, c.diagnostic)")
 replaceFile(client, "\tif _, err = c.conn.CreateOffer(medias); err != nil {", "\tc.diagnostic.Stage(\"create_offer\")\n\tif _, err = c.conn.CreateOffer(medias); err != nil {")
 replaceFile(client, "\tif err = c.SendSDPtoRobot(offer); err != nil {", "\tc.diagnostic.Stage(\"send_offer\")\n\tif err = c.SendSDPtoRobot(offer); err != nil {")
 replaceFile(client, "\tts := time.Now().Add(time.Second * 5)", "\tc.diagnostic.Stage(\"get_answer\")\n\tts := time.Now().Add(time.Second * 5)")
 replaceFile(client, "\t\tif desc, _ := c.GetDeviceSDP(); desc != nil {", "\t\tdesc, answerErr := c.GetDeviceSDP()\n\t\tc.diagnostic.Note(answerErr)\n\t\tif desc != nil {")
 replaceFile(client, "\t\t\tif err = c.conn.SetAnswer(desc.SDP); err != nil {", "\t\t\tc.diagnostic.Stage(\"set_answer\")\n\t\t\tif err = c.conn.SetAnswer(desc.SDP); err != nil {")
 replaceFile(client, "\t\t\t\t_ = c.SendICEtoRobot(msg.ToJSON().Candidate, \"0\")", "\t\t\t\tc.diagnostic.Note(c.SendICEtoRobot(msg.ToJSON().Candidate, \"0\"))")
 replaceFile(client, "\t\t\tif ice, _ := c.GetDeviceICE(); ice != nil {", "\t\t\tice, iceErr := c.GetDeviceICE()\n\t\t\tc.diagnostic.Note(iceErr)\n\t\t\tif ice != nil {")
 replaceFile(client, "\t\t\t\t\t_ = c.conn.AddCandidate(candidate)", "\t\t\t\t\tc.diagnostic.Note(c.conn.AddCandidate(candidate))")
 replaceFile(client, "\tticker := time.NewTicker(time.Second * 2)", "\tc.diagnostic.Stage(\"wait_connected\")\n\tticker := time.NewTicker(time.Second * 2)")
 replaceFile(client, "\t\t\treturn nil\n\t\t}\n\t}\n}", "\t\t\tc.diagnostic.Stage(\"connected\")\n\t\t\treturn nil\n\t\t}\n\t}\n}")
 const iot = "pkg/roborock/iot/client.go"
 replaceFile(iot, `"github.com/AlexxIT/go2rtc/pkg/mqtt"`, `"github.com/AlexxIT/go2rtc/pkg/mqtt"`+"\n\t\"github.com/AlexxIT/go2rtc/pkg/roborock/diagnostic\"")
 replaceFile(iot, "func Dial(rawURL string) (*rpc.Client, error) {", `func Dial(rawURL string) (*rpc.Client, error) { return DialWithDiagnostic(rawURL, nil) }

 func DialWithDiagnostic(rawURL string, evidence *diagnostic.Attempt) (*rpc.Client, error) {`)
 replaceFile(iot, "\tconn, err := net.DialTimeout", "\tevidence.Stage(\"tcp\")\n\tconn, err := net.DialTimeout")
 replaceFile(iot, "\tconf := &tls.Config", "\tevidence.Stage(\"tls\")\n\tconf := &tls.Config")
 replaceFile(iot, "\tmqtt *mqtt.Client", "\tmqtt *mqtt.Client\n\tdiagnostic *diagnostic.Attempt")
 replaceFile(iot, "\t\t_, payload, err := c.mqtt.Read()\n\t\tif err != nil {", "\t\t_, payload, err := c.mqtt.Read()\n\t\tif err != nil {\n\t\t\tc.diagnostic.Note(err)")
 replaceFile(iot, "\t\tmqtt:     mqtt.NewClient(sconn),", "\t\tmqtt:     mqtt.NewClient(sconn),\n\t\tdiagnostic: evidence,")
 replaceFile(iot, "\tif err = c.mqtt.Connect(cameraMQTTClientID(), user, pass); err != nil {", "\tevidence.Stage(\"mqtt\")\n\terr = c.mqtt.Connect(cameraMQTTClientID(), user, pass)\n\tevidence.Result(\"mqtt\", err)\n\tif err != nil {")
 replaceFile(iot, "\tif err = c.mqtt.Subscribe(\"rr/m/o/\" + c.devTopic); err != nil {", "\tevidence.Stage(\"subscribe\")\n\tif err = c.mqtt.Subscribe(\"rr/m/o/\" + c.devTopic); err != nil {")
 const conn = "pkg/webrtc/conn.go"
 replaceFile(iot, "\t\tif res.Error.Code != 0 {\n\t\t\tr.Error = res.Error.Message", "\t\tif res.Error.Code != 0 {\n\t\t\tc.diagnostic.VendorError(res.Error.Code, res.Error.Message)\n\t\t\tr.Error = res.Error.Message")
 replaceFile(conn, `"github.com/AlexxIT/go2rtc/pkg/core"`, `"github.com/AlexxIT/go2rtc/pkg/core"`+"\n\t\"github.com/AlexxIT/go2rtc/pkg/roborock/diagnostic\"")
 replaceFile(conn, "\tpc *webrtc.PeerConnection", "\tpc *webrtc.PeerConnection\n\tdiagnostic *diagnostic.Attempt")
 replaceFile(conn, "func NewConn(pc *webrtc.PeerConnection) *Conn {", `func NewConn(pc *webrtc.PeerConnection) *Conn { return NewConnWithDiagnostic(pc, nil) }

 func NewConnWithDiagnostic(pc *webrtc.PeerConnection, evidence *diagnostic.Attempt) *Conn {`)
 replaceFile(conn, "\t\tpc: pc,", "\t\tpc: pc,\n\t\tdiagnostic: evidence,")
 replaceFile(conn, "\tpc.OnICECandidate(func(candidate *webrtc.ICECandidate) {", ` if evidence != nil {
 evidence.Peer(pc)
 pc.SCTP().Transport().OnStateChange(func(state webrtc.DTLSTransportState){ evidence.DTLS(state) })
 }
 pc.OnICECandidate(func(candidate *webrtc.ICECandidate) {`)
 replaceFile(conn, "\tpc.OnICEConnectionStateChange(func(state webrtc.ICEConnectionState) {", "\tpc.OnICEConnectionStateChange(func(state webrtc.ICEConnectionState) {\n\t\tevidence.ICE(state)")
 replaceFile(conn, "\t\t\tfunc(pair *webrtc.ICECandidatePair) {", "\t\t\tfunc(pair *webrtc.ICECandidatePair) {\n\t\t\t\tevidence.Pair(pair)")
 replaceFile(conn, "\tpc.OnTrack(func(remote *webrtc.TrackRemote, receiver *webrtc.RTPReceiver) {", "\tpc.OnTrack(func(remote *webrtc.TrackRemote, receiver *webrtc.RTPReceiver) {\n\t\tvideo := remote.Kind() == webrtc.RTPCodecTypeVideo\n\t\tif video { evidence.Track() }")
 replaceFile(conn, "\t\tif media == nil {\n\t\t\treturn", "\t\tif media == nil {\n\t\t\tif video { evidence.Reader(\"unmapped_track\") }\n\t\t\treturn")
 replaceFile(conn, "\t\ttrack, err := c.GetTrack(media, codec)\n\t\tif err != nil {\n\t\t\treturn", "\t\ttrack, err := c.GetTrack(media, codec)\n\t\tif err != nil {\n\t\t\tif video { evidence.Reader(\"track_error\") }\n\t\t\treturn")
 replaceFile(conn, "\t\t\tn, _, err := remote.Read(b)\n\t\t\tif err != nil {\n\t\t\t\treturn", "\t\t\tn, _, err := remote.Read(b)\n\t\t\tif err != nil {\n\t\t\t\tif video { evidence.ReadError(err) }\n\t\t\t\treturn")
 replaceFile(conn, "\t\t\tif err := packet.Unmarshal(b[:n]); err != nil {\n\t\t\t\treturn", "\t\t\tif err := packet.Unmarshal(b[:n]); err != nil {\n\t\t\t\tif video { evidence.Reader(\"rtp_error\") }\n\t\t\t\treturn")
 replaceFile(conn, "\t\t\tif len(packet.Payload) == 0 {", "\t\t\tif video { evidence.Packet(n) }\n\t\t\tif len(packet.Payload) == 0 {")
 replaceFile(conn, "\tpc.OnConnectionStateChange(func(state webrtc.PeerConnectionState) {", "\tpc.OnConnectionStateChange(func(state webrtc.PeerConnectionState) {\n\t\tevidence.PC(state)")
 replaceFile("pkg/roborock/producer.go", "\t_ = c.iot.Close()", "\tc.conn.RetainDiagnosticBeforeClose()\n\t_ = c.iot.Close()")
 replaceFile(conn, "func (c *Conn) Close() error {", "func (c *Conn) RetainDiagnosticBeforeClose() { c.diagnostic.BeforeClose(c.pc) }\n\nfunc (c *Conn) Close() error {\n\tc.RetainDiagnosticBeforeClose()")
 const internal = "internal/roborock/roborock.go"
 replaceFile(internal, `"github.com/AlexxIT/go2rtc/pkg/roborock"`, `"github.com/AlexxIT/go2rtc/pkg/roborock"`+"\n\t\"github.com/AlexxIT/go2rtc/pkg/roborock/diagnostic\"")
 replaceFile(internal, "\tapi.HandleFunc(\"api/roborock\", apiHandle)", ` api.HandleFunc("api/roborock", apiHandle)
 api.HandleFunc("api/roborock/diagnostic", diagnosticHandle)`)
 b, err := os.ReadFile(internal); if err != nil { panic(err) }
 writeFile(internal, string(b)+`
 func diagnosticHandle(w http.ResponseWriter, r *http.Request) {
 if r.Method != http.MethodGet { w.WriteHeader(http.StatusMethodNotAllowed); return }
 api.ResponseJSON(w, diagnostic.Latest())
 }
 `)
 fmt.Println("Applied retained credential-free acquisition diagnostics")
 }

 const diagnosticSource = `package diagnostic

 import (
 "errors"
 "io"
 "net"
 "net/rpc"
 "sync"
 "time"
 pion "github.com/pion/webrtc/v4"
 )

 // No URLs, identifiers, SDP, credentials, error strings or candidate addresses
 // are ever stored here. All string fields pass closed enum mappings.
 type Snapshot struct {
 Available bool ` + "`json:\"available\"`" + `
 Attempt uint64 ` + "`json:\"attempt\"`" + `
 ElapsedMs int64 ` + "`json:\"elapsedMs\"`" + `
 Stage string ` + "`json:\"stage\"`" + `
 Failure string ` + "`json:\"failure\"`" + `
 VendorCode int ` + "`json:\"vendorCode\"`" + `
 VendorReason string ` + "`json:\"vendorReason\"`" + `
 MQTT string ` + "`json:\"mqtt\"`" + `
 PIN string ` + "`json:\"pin\"`" + `
 TURN string ` + "`json:\"turn\"`" + `
 PCState string ` + "`json:\"pcState\"`" + `
 ICEState string ` + "`json:\"iceState\"`" + `
 DTLSState string ` + "`json:\"dtlsState\"`" + `
 LocalCandidateType string ` + "`json:\"localCandidateType\"`" + `
 RemoteCandidateType string ` + "`json:\"remoteCandidateType\"`" + `
 CandidateProtocol string ` + "`json:\"candidateProtocol\"`" + `
 VideoTracks uint64 ` + "`json:\"videoTracks\"`" + `
 VideoPackets uint64 ` + "`json:\"videoPackets\"`" + `
 VideoBytes uint64 ` + "`json:\"videoBytes\"`" + `
 ReaderExit string ` + "`json:\"readerExit\"`" + `
 }
 func empty() Snapshot { return Snapshot{Stage:"none",Failure:"none",VendorReason:"none",MQTT:"unknown",PIN:"unknown",TURN:"unknown",PCState:"unknown",ICEState:"unknown",DTLSState:"unknown",LocalCandidateType:"unknown",RemoteCandidateType:"unknown",CandidateProtocol:"unknown",ReaderExit:"none"} }
 type Attempt struct { mu sync.Mutex; value Snapshot; start, end time.Time; closing bool }
 var retained struct { sync.Mutex; sequence uint64; latest *Attempt }
 func Begin() *Attempt {
 retained.Lock(); defer retained.Unlock()
 retained.sequence++
 a:=&Attempt{value:empty(),start:time.Now()}; a.value.Available=true; a.value.Attempt=retained.sequence
 retained.latest=a
 return a
 }
 func Latest() Snapshot { retained.Lock(); a:=retained.latest; retained.Unlock(); if a==nil { return empty() }; return a.Snapshot() }
 func (a *Attempt) Snapshot() Snapshot { if a==nil { return empty() }; a.mu.Lock(); defer a.mu.Unlock(); v:=a.value; end:=a.end; if end.IsZero(){end=time.Now()}; v.ElapsedMs=end.Sub(a.start).Milliseconds(); return v }
 func enum(s string, allowed ...string) string { for _,v:=range allowed { if s==v {return s} }; return "unknown" }
 func (a *Attempt) Stage(stage string) { if a==nil {return}; a.mu.Lock(); defer a.mu.Unlock(); a.value.Stage=enum(stage,"parse_url","tcp","tls","mqtt","subscribe","pin","status","stop_preview","start_preview","turn","create_api","create_peer","create_offer","send_offer","get_answer","set_answer","wait_connected","connected") }
 func kind(err error) string { if err==nil{return "ok"}; if errors.Is(err,io.EOF)||errors.Is(err,io.ErrUnexpectedEOF){return "eof"}; if errors.Is(err,net.ErrClosed)||errors.Is(err,rpc.ErrShutdown){return "closed"}; var n net.Error; if errors.As(err,&n) && n.Timeout(){return "timeout"}; if err.Error()=="wrong login" || err.Error()=="wrong pin code" {return "rejected"}; var server rpc.ServerError; if errors.As(err,&server){return "rpc_error"}; return "error" }
 func (a *Attempt) Result(field string, err error) { if a==nil{return}; a.mu.Lock(); defer a.mu.Unlock(); v:=kind(err); switch field {case "mqtt":a.value.MQTT=v;case "pin":a.value.PIN=v;case "turn":a.value.TURN=v} }
 func (a *Attempt) VendorError(code int, message string) {
 if a==nil{return}; a.mu.Lock(); defer a.mu.Unlock(); if a.closing{return}
 a.value.VendorCode=code; a.value.VendorReason="unknown"
 switch message {
 case "permission denied", "Permission denied", "permission_denied": a.value.VendorReason="permission_denied"
 case "busy", "camera busy", "device busy": a.value.VendorReason="busy"
 case "invalid params", "invalid parameter", "invalid parameters": a.value.VendorReason="invalid_params"
 case "not supported", "method not found": a.value.VendorReason="unsupported"
 case "not allowed", "forbidden": a.value.VendorReason="not_allowed"
 case "camera disabled", "home security disabled": a.value.VendorReason="camera_disabled"
 }
 }
 func (a *Attempt) SkipPIN(){if a==nil{return};a.mu.Lock();a.value.PIN="skipped";a.mu.Unlock()}
 func (a *Attempt) Note(err error){if a==nil||err==nil{return};a.mu.Lock();if !a.closing&&a.value.Failure=="none"{a.value.Failure=failureKind(err)};a.mu.Unlock()}
 func failureKind(err error)string{switch kind(err){case "ok":return "none";case "rejected":if err.Error()=="wrong login"{return "mqtt_rejected"};return "pin_rejected";default:return kind(err)}}
 func (a *Attempt) Finish(err error){
 if a==nil{return};a.mu.Lock();defer a.mu.Unlock();a.end=time.Now()
 if err==nil{a.value.Failure="none";return}
 if a.value.Stage=="parse_url"{a.value.Failure="invalid_url";return}
 if err.Error()=="camera not ready"{a.value.Failure="camera_not_ready";return}
 if err.Error()=="can't get device SDP"{if a.value.Failure=="none"{a.value.Failure="answer_timeout"};return}
 if err.Error()=="can't connect"{if a.value.Failure=="none"{a.value.Failure="connect_failed"};return}
 if kind(err)=="closed"&&a.value.Failure!="none"{return}
 a.value.Failure=failureKind(err)
 }
 func (a *Attempt) PC(v pion.PeerConnectionState){if a==nil{return};a.mu.Lock();defer a.mu.Unlock();if !a.closing {a.value.PCState=enum(v.String(),"new","connecting","connected","disconnected","failed","closed")}}
 func (a *Attempt) ICE(v pion.ICEConnectionState){if a==nil{return};a.mu.Lock();defer a.mu.Unlock();if !a.closing {a.value.ICEState=enum(v.String(),"new","checking","connected","completed","disconnected","failed","closed")}}
 func (a *Attempt) DTLS(v pion.DTLSTransportState){if a==nil{return};a.mu.Lock();defer a.mu.Unlock();if !a.closing {a.value.DTLSState=enum(v.String(),"new","connecting","connected","closed","failed")}}
 func (a *Attempt) Peer(pc *pion.PeerConnection){if a==nil||pc==nil{return};a.PC(pc.ConnectionState());a.ICE(pc.ICEConnectionState());a.DTLS(pc.SCTP().Transport().State())}
 func (a *Attempt) BeforeClose(pc *pion.PeerConnection){if a==nil{return};a.Peer(pc);a.mu.Lock();a.closing=true;a.mu.Unlock()}
 func (a *Attempt) Pair(pair *pion.ICECandidatePair){if a==nil||pair==nil||pair.Local==nil||pair.Remote==nil{return};a.mu.Lock();defer a.mu.Unlock();if a.closing{return};a.value.LocalCandidateType=enum(pair.Local.Typ.String(),"host","srflx","prflx","relay");a.value.RemoteCandidateType=enum(pair.Remote.Typ.String(),"host","srflx","prflx","relay");a.value.CandidateProtocol=enum(pair.Remote.Protocol.String(),"udp","tcp")}
 func (a *Attempt) Track(){if a==nil{return};a.mu.Lock();a.value.VideoTracks++;a.mu.Unlock()}
 func (a *Attempt) Packet(n int){if a==nil||n<0{return};a.mu.Lock();a.value.VideoPackets++;a.value.VideoBytes+=uint64(n);a.mu.Unlock()}
 func (a *Attempt) Reader(exit string){if a==nil{return};a.mu.Lock();a.value.ReaderExit=enum(exit,"eof","timeout","closed","read_error","rtp_error","unmapped_track","track_error");a.mu.Unlock()}
 func (a *Attempt) ReadError(err error){switch{case errors.Is(err,io.EOF)||errors.Is(err,io.ErrUnexpectedEOF):a.Reader("eof");case errors.Is(err,net.ErrClosed):a.Reader("closed");case kind(err)=="timeout":a.Reader("timeout");default:a.Reader("read_error")}}
 `
