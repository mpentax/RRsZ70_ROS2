"""Production consumers delegate observation access; only command HTTP is transitional."""
import ast
from pathlib import Path
import pytest
from z70_manipulation.executor import Client


def test_pickup_client_delegates_observations_without_network_or_token(tmp_path,monkeypatch):
    calls=[]
    class Shared:
        def get(self,path):calls.append(('get',path));return {'shared':True}
        def frame(self):calls.append(('frame',));return 'image','front'
        def select(self,camera):calls.append(('select',camera));return {'camera':camera}
    def no_network(*args,**kwargs):raise AssertionError('independent HTTP observation')
    monkeypatch.setattr('requests.Session.request',no_network)
    client=Client('http://not-contacted.invalid',tmp_path/'does-not-exist',read_only=True,observations=Shared())
    try:
        assert client.get('/api/state')=={'shared':True}
        assert client.frame()==('image','front')
        with pytest.raises(PermissionError):client.select('front')
        with pytest.raises(PermissionError):client.rpc('get_status',[])
        assert calls==[('get','/api/state'),('frame',)]
    finally:client.close()


def test_only_driver_owns_ros_http_observation_calls():
    root=Path(__file__).resolve().parents[2]
    network=[]
    for package in root.iterdir():
        source=package/package.name
        if not source.is_dir():continue
        for path in source.rglob('*.py'):
            tree=ast.parse(path.read_text())
            for node in ast.walk(tree):
                if not isinstance(node,(ast.FunctionDef,ast.AsyncFunctionDef)):continue
                for call in ast.walk(node):
                    if not isinstance(call,ast.Call) or not isinstance(call.func,ast.Attribute):continue
                    target=ast.unparse(call.func.value)
                    if target=='requests' and call.func.attr in ('get','post','request') or target=='self.session' and call.func.attr in ('get','post','request'):
                        network.append((package.name,path.name,node.name,call.func.attr))
    assert network
    assert all(package=='z70_driver'
               for package,file,method,verb in network),network
