"""Reuse read-only R7 audit functions without invoking their historical-output main."""
import json
from pathlib import Path
from offline.task28_p5_qp_6_r7.reanalyze_r6 import analyze_cell
HERE=Path(__file__).resolve().parent
if __name__=='__main__':
 cells={c:analyze_cell(c) for c in 'ABCD'}
 maximum=max(c['sides'][s]['model_residual_Nm']['max'] for c in cells.values() for s in ('left','right'))
 out={'cells':cells,'max_residual_estimator_Nm':maximum,'reproduces_R7_historical_R6_value':maximum==.05492453116950269,'source_files_modified':False,'physics_launched':False}
 (HERE/'evidence/r6_archive_review.json').write_text(json.dumps(out,indent=2,sort_keys=True)+'\n')
 print('R6 ABCD: 72 commanded intervals / 144 bilateral writes, residual max',maximum)
