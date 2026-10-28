from offline.task28_p5_qp_5.controls import negative_controls

def test_negative_controls_fail_closed():
    c=negative_controls(); assert len(c)==5 and all(x['passed'] for x in c)
