package webrtc

import (
 "testing"
 "net"
 "time"
 "github.com/AlexxIT/go2rtc/pkg/core"
 "github.com/AlexxIT/go2rtc/pkg/roborock/diagnostic"
 "github.com/pion/rtp"
 pion "github.com/pion/webrtc/v4"
)

// Real ICE, DTLS and RTP over loopback only; no STUN, TURN, DNS or robot.
func TestAcquisitionPionLoopbackVideoEvidenceRetainedAfterClose(t *testing.T){
 m:=&pion.MediaEngine{};if err:=m.RegisterDefaultCodecs();err!=nil{t.Fatal(err)}
 s:=pion.SettingEngine{};s.SetIncludeLoopbackCandidate(true);s.SetIPFilter(func(ip net.IP)bool{return ip.String()=="127.0.0.1"});s.SetNetworkTypes([]pion.NetworkType{pion.NetworkTypeUDP4})
 api:=pion.NewAPI(pion.WithMediaEngine(m),pion.WithSettingEngine(s))
 pc,err:=api.NewPeerConnection(pion.Configuration{});if err!=nil{t.Fatal(err)}
 sender,err:=api.NewPeerConnection(pion.Configuration{});if err!=nil{t.Fatal(err)};defer sender.Close()
 a:=diagnostic.Begin();c:=NewConnWithDiagnostic(pc,a);c.Mode=core.ModeActiveProducer;defer c.Close()
 local,err:=pion.NewTrackLocalStaticRTP(pion.RTPCodecCapability{MimeType:pion.MimeTypeH264,ClockRate:90000},"SECRET_TRACK_ID","SECRET_STREAM_ID");if err!=nil{t.Fatal(err)}
 if _,err=sender.AddTrack(local);err!=nil{t.Fatal(err)}
 gathered:=pion.GatheringCompletePromise(pc)
 if _,err=c.CreateOffer([]*core.Media{{Kind:core.KindVideo,Direction:core.DirectionRecvonly}});err!=nil{t.Fatal(err)}
 select{case <-gathered:case <-time.After(5*time.Second):t.Fatal("offer gathering timeout")}
 if err=sender.SetRemoteDescription(*pc.LocalDescription());err!=nil{t.Fatal(err)}
 answer,err:=sender.CreateAnswer(nil);if err!=nil{t.Fatal(err)}
 gathered=pion.GatheringCompletePromise(sender);if err=sender.SetLocalDescription(answer);err!=nil{t.Fatal(err)}
 select{case <-gathered:case <-time.After(5*time.Second):t.Fatal("answer gathering timeout")}
 if err=c.SetAnswer(sender.LocalDescription().SDP);err!=nil{t.Fatal(err)}
 ticker:=time.NewTicker(10*time.Millisecond);defer ticker.Stop();deadline:=time.NewTimer(5*time.Second);defer deadline.Stop();var seq uint16
 for a.Snapshot().VideoPackets==0 {select{case <-ticker.C:seq++;_ = local.WriteRTP(&rtp.Packet{Header:rtp.Header{Version:2,SequenceNumber:seq,Timestamp:uint32(seq)*3000},Payload:[]byte{0x65,1,2,3}});case <-deadline.C:t.Fatalf("no real video RTP: %+v",a.Snapshot())}}
 v:=a.Snapshot();if v.VideoTracks!=1||v.VideoBytes<16||v.PCState!="connected"||v.DTLSState!="connected"||v.LocalCandidateType!="host"||v.RemoteCandidateType!="host"||v.CandidateProtocol!="udp"{t.Fatalf("missing Pion boundary: %+v",v)}
 _ = c.Close()
 deadline2:=time.Now().Add(time.Second)
 for a.Snapshot().ReaderExit=="none"&&time.Now().Before(deadline2){time.Sleep(time.Millisecond)}
 v=a.Snapshot();if v.VideoPackets==0||v.ReaderExit=="none"||v.PCState!="connected"||v.DTLSState!="connected"{t.Fatalf("close lost evidence: %+v",v)};t.Logf("retained loopback snapshot: %+v",v)
}
