import copy,unittest
from core import forward_policy,routed_config,routing_valid,TOPICS,sustained,window_metrics

class MeasurementTests(unittest.TestCase):
    def test_exact_gate_boundaries(self):
        for t in (None,):self.assertEqual(forward_policy('full-outage',10,t),(True,True))
        for condition in ('normal','full-outage','localization-fed'):
            self.assertEqual(forward_policy(condition,9.999,10),(True,True))
            self.assertEqual(forward_policy(condition,12,10),(True,True))
        self.assertEqual(forward_policy('full-outage',10,10),(False,False))
        self.assertEqual(forward_policy('full-outage',11.999,10),(False,False))
        self.assertEqual(forward_policy('localization-fed',10,10),(True,False))
        self.assertEqual(forward_policy('normal',11,10),(True,True))
        with self.assertRaises(ValueError):forward_policy('typo',11,10)

    def test_config_changes_only_scan_routes(self):
        cfg={'amcl':{'ros__parameters':{'scan_topic':'scan','transform_tolerance':1.}},
            'cost':{'topic':'/scan'},'collision_monitor':{'ros__parameters':{'source_timeout':1.,'scan':{'topic':'scan','enabled':True}}}}
        original=copy.deepcopy(cfg);r=routed_config(cfg)
        self.assertEqual(cfg,original)
        self.assertEqual(r['amcl']['ros__parameters']['scan_topic'],TOPICS['localization'])
        self.assertEqual(r['cost']['topic'],TOPICS['obstacles'])
        r['amcl']['ros__parameters']['scan_topic']='scan';r['cost']['topic']='/scan'
        r['collision_monitor']['ros__parameters']['scan']['topic']='scan'
        self.assertEqual(r,original)

    def test_graph_rejects_bypass_and_cross_feed(self):
        o='failure_lab_03_observer'
        g={'/scan':[{'node':o}],TOPICS['localization']:[{'node':n} for n in (o,'amcl')],
           TOPICS['obstacles']:[{'node':n} for n in (o,'local_costmap','global_costmap','collision_monitor')]}
        self.assertTrue(routing_valid(g))
        bad=copy.deepcopy(g);bad['/scan'].append({'node':'amcl'});self.assertFalse(routing_valid(bad))
        bad=copy.deepcopy(g);bad[TOPICS['obstacles']].append({'node':'amcl'});self.assertFalse(routing_valid(bad))

    def test_sparse_telemetry_not_sustained(self):
        self.assertIsNone(sustained([{'t':0},{'t':.4}],lambda r:True,0,1))

    def test_stop_resume_and_rotation(self):
        data=[dict(t=i*.05,speed=.1 if i<20 or i>=50 else 0.,w=0.,x=i*.01,y=0.) for i in range(100)]
        m=window_metrics(data,data,0,2,4.95)
        self.assertAlmostEqual(m['stop_latency_s'],1.)
        self.assertAlmostEqual(m['resume_after_window_s'],.5)
        for r in data:r['w']=.1
        self.assertIsNone(window_metrics(data,data,0,2,4.95)['stop_latency_s'])

    def test_missing_resume_not_zero(self):
        data=[dict(t=i*.05,speed=0.,w=0.,x=0.,y=0.) for i in range(100)]
        self.assertIsNone(window_metrics(data,data,0,2,4.95)['resume_after_window_s'])

if __name__=='__main__':unittest.main()
