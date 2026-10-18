package roborock

import (
 "encoding/json"
 "errors"
 "io"
 "net/rpc"
 "sync"
 "testing"
 "github.com/AlexxIT/go2rtc/pkg/roborock/diagnostic"
)

type fixtureReply struct{seq uint64;method string}
type fixtureCodec struct{fail string;calls chan fixtureReply;done chan struct{};once sync.Once;current fixtureReply}
func (f *fixtureCodec) WriteRequest(r *rpc.Request,_ any)error{select{case f.calls<-fixtureReply{r.Seq,r.ServiceMethod}:return nil;case <-f.done:return io.EOF}}
func (f *fixtureCodec) ReadResponseHeader(r *rpc.Response)error{select{case f.current=<-f.calls:case <-f.done:return io.EOF};if f.current.method==f.fail&&f.fail=="get_device_sdp"{return io.EOF};r.Seq=f.current.seq;if f.current.method==f.fail||f.current.method=="get_turn_server"{r.Error="SECRET_RPC_ERROR"};return nil}
func (f *fixtureCodec) ReadResponseBody(v any)error{if v==nil{return nil};switch p:=v.(type){case *string:*p=`["ok"]`;case *bool:*p=f.fail!="pin_denied";case *[]byte:switch f.current.method{case "get_homesec_connect_status":if f.fail=="stop_camera_preview"{*p=[]byte(`{"status":1,"client_id":"SECRET_CLIENT_ID"}`)}else{*p=[]byte(`{"status":0,"client_id":"none"}`)};case "get_turn_server":*p=[]byte(`{"url":"","user":"SECRET_TURN_USER","pwd":"SECRET_TURN_PASSWORD"}`);case "get_device_sdp":b,_:=json.Marshal(map[string][]byte{"dev_sdp":[]byte(`{"type":"answer","sdp":"SECRET_INVALID_SDP"}`)});*p=b}}
 return nil}
func (f *fixtureCodec) Close()error{f.once.Do(func(){close(f.done)});return nil}

func TestAcquisitionConnectFailureBoundariesInMemory(t *testing.T){
 cases:=[]struct{method,stage,failure string}{
 {"get_homesec_connect_status","status","rpc_error"},
 {"stop_camera_preview","stop_preview","rpc_error"},
 {"start_camera_preview","start_preview","rpc_error"},
 {"send_sdp_to_robot","send_offer","rpc_error"},
 {"get_device_sdp","get_answer","eof"},
 {"","set_answer","error"},
 }
 for _,tc:=range cases{t.Run(tc.stage,func(t *testing.T){f:=&fixtureCodec{fail:tc.method,calls:make(chan fixtureReply,8),done:make(chan struct{})};c:=&Client{iot:rpc.NewClientWithCodec(f),diagnostic:diagnostic.Begin()};defer c.iot.Close()
 err:=c.Connect();if err==nil{t.Fatal("fixture should fail")};if c.conn!=nil{_ = c.conn.Close()}
 v:=diagnostic.Latest();if v.Stage!=tc.stage||v.Failure!=tc.failure{t.Fatalf("expected %s/%s: %+v err=%v",tc.stage,tc.failure,v,err)}
 if tc.stage=="send_offer"||tc.stage=="get_answer"||tc.stage=="set_answer"{if v.TURN!="rpc_error"||v.PCState=="unknown"||v.ICEState=="unknown"||v.DTLSState=="unknown"{t.Fatalf("peer evidence absent: %+v",v)}}
 })}
}

func TestAcquisitionPINOutcomesWithoutNetwork(t *testing.T){
 for _,tc:=range []struct{fail,want string}{{"","ok"},{"check_homesec_password","rpc_error"},{"pin_denied","rejected"}}{f:=&fixtureCodec{fail:tc.fail,calls:make(chan fixtureReply,1),done:make(chan struct{})};c:=&Client{iot:rpc.NewClientWithCodec(f),diagnostic:diagnostic.Begin()};c.diagnostic.Stage("pin");err:=c.CheckHomesecPassword();c.diagnostic.Result("pin",err);c.diagnostic.Finish(err);_ = c.iot.Close();if diagnostic.Latest().PIN!=tc.want{t.Fatal("PIN RPC evidence")}}
 a:=diagnostic.Begin();a.Stage("pin");a.Result("pin",errors.New("wrong pin code"));a.Finish(errors.New("wrong pin code"));if diagnostic.Latest().Failure!="pin_rejected"{t.Fatal("PIN denial category")}
}
