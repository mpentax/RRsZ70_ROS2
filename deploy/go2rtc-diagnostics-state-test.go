package diagnostic

import (
 "encoding/json"
 "errors"
 "fmt"
 "io"
 "net"
 "net/rpc"
 "strings"
 "sync"
 "testing"
 pion "github.com/pion/webrtc/v4"
)

func TestAcquisitionStageFailureCategories(t *testing.T) {
 stages:=[]string{"parse_url","tcp","tls","mqtt","subscribe","pin","status","stop_preview","start_preview","turn","create_api","create_peer","create_offer","send_offer","get_answer","set_answer","wait_connected"}
 for _,stage:=range stages { t.Run(stage,func(t *testing.T){a:=Begin();a.Stage(stage);a.Finish(errors.New("SECRET_RAW_ERROR"));v:=a.Snapshot();want:="error";if stage=="parse_url"{want="invalid_url"};if v.Stage!=stage||v.Failure!=want{t.Fatalf("%+v",v)} }) }
 cases:=[]struct{stage string;err error;want string}{
 {"mqtt",errors.New("wrong login"),"mqtt_rejected"},{"pin",errors.New("wrong pin code"),"pin_rejected"},
 {"status",io.EOF,"eof"},{"start_preview",&net.DNSError{IsTimeout:true},"timeout"},
 {"send_offer",rpc.ServerError("SECRET_RPC_REPLY"),"rpc_error"},{"status",rpc.ErrShutdown,"closed"},
 {"stop_preview",errors.New("camera not ready"),"camera_not_ready"},{"get_answer",errors.New("can't get device SDP"),"answer_timeout"},{"wait_connected",errors.New("can't connect"),"connect_failed"},
 }
 for _,c:=range cases {a:=Begin();a.Stage(c.stage);err:=fmt.Errorf("SECRET_WRAPPER: %w",c.err);if c.want=="mqtt_rejected"||c.want=="pin_rejected"||c.want=="camera_not_ready"||c.want=="answer_timeout"||c.want=="connect_failed" {err=c.err};a.Finish(err);if v:=a.Snapshot();v.Failure!=c.want{t.Fatalf("%s: %+v",c.want,v)}}
 a:=Begin();a.Stage("get_answer");a.Note(io.EOF);a.Finish(errors.New("can't get device SDP"));if a.Snapshot().Failure!="eof"{t.Fatal("ignored answer RPC EOF lost")}
}

func TestAcquisitionClosedEnumSecretFreeSerialization(t *testing.T){
 a:=Begin();a.Stage("SECRET_URL");a.Result("mqtt",errors.New("SECRET_MQTT"));a.Result("pin",rpc.ServerError("SECRET_PIN"));a.Result("turn",errors.New("SECRET_TURN"));a.Reader("SECRET_READER")
 a.Pair(&pion.ICECandidatePair{Local:&pion.ICECandidate{Address:"SECRET_LOCAL",Typ:pion.ICECandidateTypeHost},Remote:&pion.ICECandidate{Address:"SECRET_REMOTE",RelatedAddress:"SECRET_RELATED",Typ:pion.ICECandidateTypeRelay,Protocol:pion.ICEProtocolUDP}})
 a.PC(pion.PeerConnectionState(255));a.ICE(pion.ICEConnectionState(255));a.DTLS(pion.DTLSTransportState(255));a.Finish(errors.New("SECRET_ERROR"))
 b,err:=json.Marshal(Latest());if err!=nil{t.Fatal(err)};if strings.Contains(string(b),"SECRET"){t.Fatalf("secret serialized: %s",b)}
 var fields map[string]any;_ = json.Unmarshal(b,&fields);if len(fields)!=20{t.Fatal("schema changed")}
 if a.Snapshot().LocalCandidateType!="host"||a.Snapshot().RemoteCandidateType!="relay"||a.Snapshot().CandidateProtocol!="udp"{t.Fatal("safe pair evidence missing")}
}

func TestAcquisitionConcurrentSnapshotRetainedSingleAttempt(t *testing.T){
 old:=Begin();a:=Begin();a.Stage("connected");var wg sync.WaitGroup
 for i:=0;i<8;i++{wg.Add(1);go func(){defer wg.Done();for j:=0;j<1000;j++{a.Track();a.Packet(1200);a.PC(pion.PeerConnectionStateConnected);a.ICE(pion.ICEConnectionStateCompleted);a.DTLS(pion.DTLSTransportStateConnected);a.Reader("eof");_,_=json.Marshal(Latest());old.Packet(999)}}()};wg.Wait();a.Finish(nil)
 v:=Latest();if v.Attempt!=a.Snapshot().Attempt||v.VideoTracks!=8000||v.VideoPackets!=8000||v.VideoBytes!=9600000{t.Fatalf("counter/snapshot: %+v",v)}
 a.BeforeClose(nil);a.PC(pion.PeerConnectionStateClosed);a.ICE(pion.ICEConnectionStateClosed);a.DTLS(pion.DTLSTransportStateClosed)
 if v:=Latest();v.PCState!="connected"||v.ICEState!="completed"||v.DTLSState!="connected"{t.Fatalf("cleanup overwrote transport evidence: %+v",v)}
 for _,err:=range []error{io.EOF,io.ErrUnexpectedEOF,net.ErrClosed,&net.DNSError{IsTimeout:true},errors.New("SECRET")} {a.ReadError(err)}
 if Latest().ReaderExit!="read_error"{t.Fatal("reader exit missing")}
}
