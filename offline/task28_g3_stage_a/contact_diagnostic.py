"""PhysX shape-contact receipts for placement diagnosis; never a D6 wrench sensor."""
import numpy as np


class ContactDiagnostic:
    def __init__(self, bridge):
        import omni.physx
        from pxr import PhysxSchema, UsdPhysics
        self.bridge=bridge
        for prim in bridge.feed.stage.Traverse():
            if str(prim.GetPath()).startswith(('/World/left_fr3/','/World/right_fr3/')) and prim.HasAPI(UsdPhysics.RigidBodyAPI):
                api=PhysxSchema.PhysxContactReportAPI.Apply(prim)
                api.CreateThresholdAttr().Set(0)
        self.subscription=omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(self.report)

    def report(self, headers, data):
        if self.bridge._task_mode!='DUAL_ATTACHED': return
        from pxr import PhysicsSchemaTools
        for h in headers:
            paths=[str(PhysicsSchemaTools.intToSdfPath(x)) for x in (h.collider0,h.collider1)]
            if not any('/left_fr3/' in p or '/right_fr3/' in p for p in paths):continue
            points=[]
            for i in range(h.contact_data_offset,h.contact_data_offset+h.num_contact_data):
                d=data[i]
                points.append({'position_world_m':list(d.position),'normal_world':list(d.normal),
                    'impulse_world_Ns':list(d.impulse),'separation_m':float(d.separation)})
            if points:
                self.bridge._log({'event':'PLACEMENT_SHAPE_CONTACT','physics_callback_step':self.bridge._step,
                    'time_s':self.bridge._time_s,'colliders':paths,'points':points,
                    'summed_impulse_world_Ns':np.sum([p['impulse_world_Ns'] for p in points],axis=0).tolist(),
                    'source':'PhysX_simulation_contact_report_after_step_not_D6'})

    def close(self):
        self.subscription.unsubscribe()
