// Retained, credential-free boundary evidence for the exact pinned upstream.
// Run after patch-go2rtc.go. Every source anchor must match exactly once.
package main

import (
 "fmt"
 "os"
 "strings"
)

func replace(path, old, next string) {
 b, err := os.ReadFile(path); if err != nil { panic(err) }
 s := string(b)
 if strings.Count(s,old)!=1 { panic("pinned diagnostic anchor changed: "+path+" anchor="+old) }
 if err := os.WriteFile(path,[]byte(strings.Replace(s,old,next,1)),0644); err != nil { panic(err) }
}
func write(path, text string) { if err:=os.WriteFile(path,[]byte(text),0644); err!=nil { panic(err) } }

func main() {
 if err:=os.MkdirAll("pkg/roborock/diagnostic",0755); err!=nil { panic(err) }
 write("pkg/roborock/diagnostic/diagnostic.go", diagnosticSource)
 const client = "pkg/roborock/client.go"
 replace(client, `"github.com/AlexxIT/go2rtc/pkg/roborock/iot"`, `"github.com/AlexxIT/go2rtc/pkg/roborock/iot"`+"\n\t\"github.com/AlexxIT/go2rtc/pkg/roborock/diagnostic\"")
 replace(client, "\turl string", "\turl string\n\tdiagnostic *diagnostic.Attempt")
 replace(client,"func (c *Client) Dial() error {",`func (c *Client) Dial() (err error) {
 c.diagnostic = diagnostic.Begin()
 defer func(){ if err != nil { c.diagnostic.Finish(err) } }()
 c.diagnostic.Stage("parse_url")`)
 replace(client,"iot.Dial(c.url)","iot.DialWithDiagnostic(c.url, c.diagnostic)")
 replace(client,"\tc.pin = u.Query().Get(\"pin\")", "\tc.diagnostic.Stage(\"pin\")\n\tc.pin = u.Query().Get(\"pin\")")
 replace(client,"\t\treturn c.CheckHomesecPassword()", "\t\terr = c.CheckHomesecPassword()\n\t\tc.diagnostic.Result(\"pin\", err)\n\t\treturn err")
 replace(client,"\treturn nil\n}\n\nfunc (c *Client) Connect() error {", ` c.diagnostic.SkipPIN()
 return nil
}

func (c *Client) Connect() (err error) {
 defer func(){ c.diagnostic.Finish(err) }()`)
 replace(client,"\t\tclientID, err := c.GetHomesecConnectStatus()", "\t\tc.diagnostic.Stage(\"status\")\n\t\tclientID, err := c.GetHomesecConnectStatus()")
 replace(client,"\t\tif err = c.StopCameraPreview(clientID); err != nil {", "\t\tc.diagnostic.Stage(\"stop_preview\")\n\t\tif err = c.StopCameraPreview(clientID); err != nil {")
 replace(client,"\tif err := c.StartCameraPreview(); err != nil {", "\tc.diagnostic.Stage(\"start_preview\")\n\tif err := c.StartCameraPreview(); err != nil {")
 replace(client,"\tif turn, _ := c.GetTurnServer(); turn != nil {", "\tc.diagnostic.Stage(\"turn\")\n\tturn, turnErr := c.GetTurnServer()\n\tc.diagnostic.Result(\"turn\", turnErr)\n\tif turn != nil {")
 replace(client,"\tapi, err := webrtc.NewAPI()", "\tc.diagnostic.Stage(\"create_api\")\n\tapi, err := webrtc.NewAPI()")
 replace(client,"\tpc, err := api.NewPeerConnection(conf)", "\tc.diagnostic.Stage(\"create_peer\")\n\tpc, err := api.NewPeerConnection(conf)")
 replace(client,"\tc.conn = webrtc.NewConn(pc)", "\tc.conn = webrtc.NewConnWithDiagnostic(pc, c.diagnostic)")
 replace(client,"\tif _, err = c.conn.CreateOffer(medias); err != nil {", "\tc.diagnostic.Stage(\"create_offer\")\n\tif _, err = c.conn.CreateOffer(medias); err != nil {")
 replace(client,"\tif err = c.SendSDPtoRobot(offer); err != nil {", "\tc.diagnostic.Stage(\"send_offer\")\n\tif err = c.SendSDPtoRobot(offer); err != nil {")
 replace(client,"\tts := time.Now().Add(time.Second * 5)", "\tc.diagnostic.Stage(\"get_answer\")\n\tts := time.Now().Add(time.Second * 5)")
 replace(client,"\t\tif desc, _ := c.GetDeviceSDP(); desc != nil {", "\t\tdesc, answerErr := c.GetDeviceSDP()\n\t\tc.diagnostic.Note(answerErr)\n\t\tif desc != nil {")
 replace(client,"\t\t\tif err = c.conn.SetAnswer(desc.SDP); err != nil {",  "\t\t\tc.diagnostic.Stage(\"set_answer\")\n\t\t\tif err = c.conn.SetAnswer(desc.SDP); err != nil {")
 replace(client,"\t\t\t\t_ = c.SendICEtoRobot(msg.ToJSON().Candidate, \"0\")", "\t\t\t\tc.diagnostic.Note(c.SendICEtoRobot(msg.ToJSON().Candidate, \"0\"))")
 replace(client,"\t\t\tif ice, _ := c.GetDeviceICE(); ice != nil {", "\t\t\tice, iceErr := c.GetDeviceICE()\n\t\t\tc.diagnostic.Note(iceErr)\n\t\t\tif ice != nil {")
 replace(client,"\t\t\t\t\t_ = c.conn.AddCandidate(candidate)", "\t\t\t\t\tc.diagnostic.Note(c.conn.AddCandidate(candidate))")
 replace(client,"\tticker := time.NewTicker(time.Second * 2)", "\tc.diagnostic.Stage(\"wait_connected\")\n\tticker := time.NewTicker(time.Second * 2)")
 replace(client,"\t\t\treturn nil\n\t\t}\n\t}\n}", "\t\t\tc.diagnostic.Stage(\"connected\")\n\t\t\treturn nil\n\t\t}\n\t}\n}")
 const iot = "pkg/roborock/iot/client.go"
 replace(iot, `"github.com/AlexxIT/go2rtc/pkg/mqtt"`, `"github.com/AlexxIT/go2rtc/pkg/mqtt"`+"\n\t\"github.com/AlexxIT/go2rtc/pkg/roborock/diagnostic\"")
 replace(iot,"func Dial(rawURL string) (*rpc.Client, error) {",`func Dial(rawURL string) (*rpc.Client, error) { return DialWithDiagnostic(rawURL, nil) }

func DialWithDiagnostic(rawURL string, evidence *diagnostic.Attempt) (*rpc.Client, error) {`)
 replace(iot,"\tconn, err := net.DialTimeout", "\tevidence.Stage(\"tcp\")\n\tconn, err := net.DialTimeout")
 replace(iot,"\tconf := &tls.Config", "\tevidence.Stage(\"tls\")\n\tconf := &tls.Config")
 replace(iot,"\tmqtt *mqtt.Client", "\tmqtt *mqtt.Client\n\tdiagnostic *diagnostic.Attempt")
 replace(iot,"\t\t_, payload, err := c.mqtt.Read()\n\t\tif err != nil {", "\t\t_, payload, err := c.mqtt.Read()\n\t\tif err != nil {\n\t\t\tc.diagnostic.Note(err)")
 replace(iot,"\t\tmqtt:     mqtt.NewClient(sconn),", "\t\tmqtt:     mqtt.NewClient(sconn),\n\t\tdiagnostic: evidence,")
 replace(iot,"\tif err = c.mqtt.Connect(cameraMQTTClientID(), user, pass); err != nil {", "\tevidence.Stage(\"mqtt\")\n\terr = c.mqtt.Connect(cameraMQTTClientID(), user, pass)\n\tevidence.Result(\"mqtt\", err)\n\tif err != nil {")
 replace(iot,"\tif err = c.mqtt.Subscribe(\"rr/m/o/\" + c.devTopic); err != nil {", "\tevidence.Stage(\"subscribe\")\n\tif err = c.mqtt.Subscribe(\"rr/m/o/\" + c.devTopic); err != nil {")
 const conn = "pkg/webrtc/conn.go"
 replace(iot, "\t\tif res.Error.Code != 0 {\n\t\t\tr.Error = res.Error.Message", "\t\tif res.Error.Code != 0 {\n\t\t\tc.diagnostic.VendorError(res.Error.Code, res.Error.Message)\n\t\t\tr.Error = res.Error.Message")
 replace(conn, `"github.com/AlexxIT/go2rtc/pkg/core"`, `"github.com/AlexxIT/go2rtc/pkg/core"`+"\n\t\"github.com/AlexxIT/go2rtc/pkg/roborock/diagnostic\"")
 replace(conn,"\tpc *webrtc.PeerConnection", "\tpc *webrtc.PeerConnection\n\tdiagnostic *diagnostic.Attempt")
 replace(conn,"func NewConn(pc *webrtc.PeerConnection) *Conn {",`func NewConn(pc *webrtc.PeerConnection) *Conn { return NewConnWithDiagnostic(pc, nil) }

func NewConnWithDiagnostic(pc *webrtc.PeerConnection, evidence *diagnostic.Attempt) *Conn {`)
 replace(conn,"\t\tpc: pc,", "\t\tpc: pc,\n\t\tdiagnostic: evidence,")
 replace(conn,"\tpc.OnICECandidate(func(candidate *webrtc.ICECandidate) {", ` if evidence != nil {
  evidence.Peer(pc)
  pc.SCTP().Transport().OnStateChange(func(state webrtc.DTLSTransportState){ evidence.DTLS(state) })
 }
 pc.OnICECandidate(func(candidate *webrtc.ICECandidate) {`)
 replace(conn,"\tpc.OnICEConnectionStateChange(func(state webrtc.ICEConnectionState) {", "\tpc.OnICEConnectionStateChange(func(state webrtc.ICEConnectionState) {\n\t\tevidence.ICE(state)")
 replace(conn,"\t\t\tfunc(pair *webrtc.ICECandidatePair) {", "\t\t\tfunc(pair *webrtc.ICECandidatePair) {\n\t\t\t\tevidence.Pair(pair)")
 replace(conn,"\tpc.OnTrack(func(remote *webrtc.TrackRemote, receiver *webrtc.RTPReceiver) {", "\tpc.OnTrack(func(remote *webrtc.TrackRemote, receiver *webrtc.RTPReceiver) {\n\t\tvideo := remote.Kind() == webrtc.RTPCodecTypeVideo\n\t\tif video { evidence.Track() }")
 replace(conn,"\t\tif media == nil {\n\t\t\treturn", "\t\tif media == nil {\n\t\t\tif video { evidence.Reader(\"unmapped_track\") }\n\t\t\treturn")
 replace(conn,"\t\ttrack, err := c.GetTrack(media, codec)\n\t\tif err != nil {\n\t\t\treturn", "\t\ttrack, err := c.GetTrack(media, codec)\n\t\tif err != nil {\n\t\t\tif video { evidence.Reader(\"track_error\") }\n\t\t\treturn")
 replace(conn,"\t\t\tn, _, err := remote.Read(b)\n\t\t\tif err != nil {\n\t\t\t\treturn", "\t\t\tn, _, err := remote.Read(b)\n\t\t\tif err != nil {\n\t\t\t\tif video { evidence.ReadError(err) }\n\t\t\t\treturn")
 replace(conn,"\t\t\tif err := packet.Unmarshal(b[:n]); err != nil {\n\t\t\t\treturn", "\t\t\tif err := packet.Unmarshal(b[:n]); err != nil {\n\t\t\t\tif video { evidence.Reader(\"rtp_error\") }\n\t\t\t\treturn")
 replace(conn,"\t\t\tif len(packet.Payload) == 0 {", "\t\t\tif video { evidence.Packet(n) }\n\t\t\tif len(packet.Payload) == 0 {")
 replace(conn,"\tpc.OnConnectionStateChange(func(state webrtc.PeerConnectionState) {", "\tpc.OnConnectionStateChange(func(state webrtc.PeerConnectionState) {\n\t\tevidence.PC(state)")
 replace("pkg/roborock/producer.go", "\t_ = c.iot.Close()", "\tc.conn.RetainDiagnosticBeforeClose()\n\t_ = c.iot.Close()")
 replace(conn,"func (c *Conn) Close() error {", "func (c *Conn) RetainDiagnosticBeforeClose() { c.diagnostic.BeforeClose(c.pc) }\n\nfunc (c *Conn) Close() error {\n\tc.RetainDiagnosticBeforeClose()")
 const internal = "internal/roborock/roborock.go"
 replace(internal,`"github.com/AlexxIT/go2rtc/pkg/roborock"`,`"github.com/AlexxIT/go2rtc/pkg/roborock"`+"\n\t\"github.com/AlexxIT/go2rtc/pkg/roborock/diagnostic\"")
 replace(internal,"\tapi.HandleFunc(\"api/roborock\", apiHandle)",` api.HandleFunc("api/roborock", apiHandle)
 api.HandleFunc("api/roborock/diagnostic", diagnosticHandle)`)
 b,err:=os.ReadFile(internal); if err!=nil { panic(err) }
 write(internal,string(b)+`
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
