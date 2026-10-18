package iot

import (
 "io"
 "encoding/json"
 "encoding/binary"
 "hash/crc32"
 "net"
 "net/rpc"
 "testing"
 "time"
 "github.com/AlexxIT/go2rtc/pkg/mqtt"
 "github.com/AlexxIT/go2rtc/pkg/roborock/diagnostic"
)

type failureConn struct{err error}

func TestAcquisitionVendorDenialCodeRetainedWithoutRawMessage(t *testing.T) {
 for _,tc:=range []struct{message,want string}{{"permission denied","permission_denied"},{"SECRET_MESSAGE_URL_PIN","unknown"}} {
  left,right:=net.Pipe();defer left.Close();defer right.Close()
  a:=diagnostic.Begin();a.Stage("start_preview")
  c:=&Codec{mqtt:mqtt.NewClient(left),devKey:"0123456789abcdef",diagnostic:a}
  res:=response{ID:1};res.Error.Code=-10003;res.Error.Message=tc.message
  rb,err:=json.Marshal(res);if err!=nil {t.Fatal(err)}
  msg:=dps{};msg.Dps.Res=string(rb);b,err:=json.Marshal(msg);if err!=nil {t.Fatal(err)}
  payload:=c.Encrypt(b,1,1,1)
  binary.BigEndian.PutUint16(payload[15:],102)
  binary.BigEndian.PutUint32(payload[len(payload)-4:],crc32.ChecksumIEEE(payload[:len(payload)-4]))
  sent:=make(chan error,1);go func(){
   err:=mqtt.NewClient(right).Publish("rr/m/o/fixture",payload)
   if err==nil {_,err=io.ReadFull(right,make([]byte,4))}
   sent<-err
  }()
  header:=rpc.Response{};if err=c.ReadResponseHeader(&header);err!=nil {t.Fatal(err)}
  if err=<-sent;err!=nil {t.Fatal(err)}
  if header.Error!=tc.message {t.Fatal("RPC behavior changed")}
  b,err=json.Marshal(diagnostic.Latest());if err!=nil {t.Fatal(err)}
  var v map[string]any;_ = json.Unmarshal(b,&v)
  if v["vendorCode"]!=float64(-10003)||v["vendorReason"]!=tc.want {t.Fatalf("lost safe vendor denial: %s",b)}
  if tc.want=="unknown" {for _,value:=range v {if value==tc.message {t.Fatal("raw message leaked")}}}
 }
}

func(c failureConn)Read([]byte)(int,error){return 0,c.err}
func(c failureConn)Write(b []byte)(int,error){return len(b),nil}
func(c failureConn)Close()error{return nil}
func(c failureConn)LocalAddr()net.Addr{return nil}
func(c failureConn)RemoteAddr()net.Addr{return nil}
func(c failureConn)SetDeadline(time.Time)error{return nil}
func(c failureConn)SetReadDeadline(time.Time)error{return nil}
func(c failureConn)SetWriteDeadline(time.Time)error{return nil}

func TestAcquisitionMQTTReadFailureBeforeRPCShutdownRetained(t *testing.T){
 for _,tc:=range []struct{err error;want string}{{io.EOF,"eof"},{&net.DNSError{IsTimeout:true},"timeout"}}{a:=diagnostic.Begin();a.Stage("status");c:=&Codec{mqtt:mqtt.NewClient(failureConn{tc.err}),diagnostic:a};if err:=c.ReadResponseHeader(&rpc.Response{});err!=tc.err{t.Fatal("read behavior changed")};a.Finish(rpc.ErrShutdown);if v:=diagnostic.Latest();v.Failure!=tc.want||v.Stage!="status"{t.Fatalf("underlying MQTT read lost: %+v",v)}}
 a:=diagnostic.Begin();a.Stage("connected");a.Finish(nil);a.BeforeClose(nil);c:=&Codec{mqtt:mqtt.NewClient(failureConn{io.EOF}),diagnostic:a};_ = c.ReadResponseHeader(&rpc.Response{});if diagnostic.Latest().Failure!="none"{t.Fatal("intentional cleanup became acquisition failure")}
}
