"""Read-only independent process audit, including initialization without result.json."""
import json
from pathlib import Path
HERE=Path(__file__).resolve().parent

def audit(name):
 p=HERE/'evidence/process'/name/'lifecycle.json'
 if not p.exists():return {'status':'NOT_RUN','verified':False}
 life=json.loads(p.read_text());child=HERE/'evidence/diagnostic'/name/'child_lifecycle.jsonl'
 events=[json.loads(x) for x in child.read_text().splitlines()] if child.exists() else []
 cache=next((e for e in events if e['event']=='CACHE_ISOLATION_VERIFIED'),None)
 root=life.get('portable_root','')
 cache_valid=bool(cache and all(cache['tokens'].get(k,'').startswith(root+'/') for k in ('cache','appcache','logs','data')))
 exited=bool(life.get('status')=='EXITED' and life.get('residual_owned')==[] and life.get('portable_resources_removed') is True)
 return {'status':'AUDITED','configuration':name,'parent_lifecycle_isolation_flag':life.get('isolation_verified'),'cache_verified_before_scene':cache_valid,'exit_and_resources_verified':exited,'no_competing_processes':not life.get('preflight_competing') and not life.get('postflight_competing'),'no_kvdb_warning':not life.get('kvdb_lock_warning',True),'verified':bool(cache_valid and exited and not life.get('preflight_competing') and not life.get('postflight_competing') and not life.get('kvdb_lock_warning',True)),'exit_code':life.get('exit_code'),'signal':life.get('signal'),'normal_exit':life.get('clean_exit',False),'note':'R9 independent audit of the owned process group, cache isolation and resource cleanup.'}

if __name__=='__main__':
 out={n:audit(n) for n in ('V1','V1_RECOVERY','V2')}
 (HERE/'evidence/process_isolation_audit.json').write_text(json.dumps(out,indent=2,sort_keys=True)+'\n');print(json.dumps(out,indent=2))
