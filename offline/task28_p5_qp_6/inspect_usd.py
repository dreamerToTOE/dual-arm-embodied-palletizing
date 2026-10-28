"""Read the frozen USD with pxr only, no SimulationApp or physics."""
import argparse
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from offline.task28_p5_qp_6.common import ASSET,ASSET_SHA,HERE,dump,sha


def main():
    from pxr import Usd,UsdPhysics
    if sha(ASSET)!=ASSET_SHA: raise ValueError('USD_HASH')
    ap=argparse.ArgumentParser(); ap.add_argument('--output',type=Path,default=HERE/'evidence/usd_inspection.json'); args=ap.parse_args()
    stage=Usd.Stage.Open(str(ASSET)); records=[]
    for prim in stage.Traverse():
        attributes={}
        for attr in prim.GetAttributes():
            name=attr.GetName()
            if any(s in name.lower() for s in ['mimic','friction','armature','mass','inertia','lowerlimit','upperlimit','axis','maxforce','centerofmass']):
                value=attr.Get()
                attributes[name]=None if value is None else str(value)
        schemas=list(prim.GetAppliedSchemas())
        if attributes or any('Mimic' in s for s in schemas): records.append({'path':str(prim.GetPath()),'type':prim.GetTypeName(),'schemas':schemas,'authored_api_schema_metadata':str(prim.GetMetadata('apiSchemas')),'relationships':{rel.GetName():[str(t) for t in rel.GetTargets()] for rel in prim.GetRelationships()},'attributes':attributes})
    dump(args.output,{'asset_sha':sha(ASSET),'pxr_version':Usd.GetVersion(),'records':records,'physics_launched':False,'uniform_cube_mass_kg':.8,'uniform_cube_com_local_m':[0,0,0],'uniform_cube_inertia_com_kg_m2':[.00192]*3,'cube_inertia_status':'ANALYTIC_UNIFORM_CUBOID_ASSUMPTION_ONLY_NOT_ACTUAL_CUBE_STAGE_VALIDATED'})


if __name__=='__main__': main()
