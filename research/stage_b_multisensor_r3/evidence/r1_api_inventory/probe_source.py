from isaacsim import SimulationApp
app=SimulationApp({'headless':True})
try:
 import sys,importlib,json
 out={}
 for name in ['omni.fabric','omni.fabric._fabric','omni.fabric._bindings','usdrt','usdrt.Rt','omni.timeline','omni.graph.core','omni.usd']:
  try:
   m=importlib.import_module(name);out[name]={'file':getattr(m,'__file__',None),'attributes':[a for a in dir(m) if any(s in a.lower() for s in ['rational','time','fabric'])]}
  except Exception as e:out[name]={'error':repr(e)}
 out['rational_attributes_in_loaded_modules']={n:[a for a in dir(m) if 'rational' in a.lower()] for n,m in list(sys.modules.items()) if m is not None and ('omni' in n or 'carb' in n or 'usdrt' in n) and any('rational' in a.lower() for a in dir(m))}
 open('/tmp/r1_fabric_api_probe.json','w').write(json.dumps(out,indent=2));print('R1_API_PROBE',json.dumps(out),flush=True)
finally:app.close()
