package roborock

import (
 "encoding/json"
 "net/http"
 "net/http/httptest"
 "testing"
 camera "github.com/AlexxIT/go2rtc/pkg/roborock"
)

// Runs unchanged against the baseline: URL parse fails before any network call.
func TestAcquisitionDiagnosticRetainsPreProducerFailure(t *testing.T) {
 http.DefaultServeMux = http.NewServeMux()
 Init()
 if _, err := camera.Dial("%SECRET_URL_PIN_TOKEN"); err == nil { t.Fatal("expected invalid URL") }
 w := httptest.NewRecorder()
 http.DefaultServeMux.ServeHTTP(w, httptest.NewRequest("GET", "/api/roborock/diagnostic", nil))
 if w.Code != http.StatusOK { t.Fatalf("retained acquisition diagnostic missing: status=%d", w.Code) }
 var v map[string]any
 if err := json.Unmarshal(w.Body.Bytes(), &v); err != nil { t.Fatal(err) }
 if v["available"] != true || v["stage"] != "parse_url" || v["failure"] != "invalid_url" { t.Fatalf("missing pre-producer boundary: %v",v) }
 allowed := map[string]bool{}
 for _, k := range []string{"available","attempt","elapsedMs","stage","failure","vendorCode","vendorReason","mqtt","pin","turn","pcState","iceState","dtlsState","localCandidateType","remoteCandidateType","candidateProtocol","videoTracks","videoPackets","videoBytes","readerExit"} { allowed[k]=true }
 if len(v) != len(allowed) { t.Fatalf("wrong schema: %v",v) }
 for k, value := range v { if !allowed[k] { t.Fatalf("unsafe field: %s",k) }; if s,ok:=value.(string); ok && s=="%SECRET_URL_PIN_TOKEN" { t.Fatal("secret serialized") } }
 w = httptest.NewRecorder()
 http.DefaultServeMux.ServeHTTP(w, httptest.NewRequest("POST", "/api/roborock/diagnostic", nil))
 if w.Code != http.StatusMethodNotAllowed { t.Fatal("diagnostic must be read-only") }
}
