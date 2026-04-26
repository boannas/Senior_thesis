python baseline_passive_lower_bound.py --mode active --plasticity none --threats 1 --food-spawn-interval 20 --food-spawn-n 1 --output-dir test_results/cond_active_threat_none

python baseline_passive_lower_bound.py --mode active --plasticity outcome --threats 1 --food-spawn-interval 20 --food-spawn-n 1 --output-dir test_results/cond_active_threat_outcome

python baseline_passive_lower_bound.py --mode active --plasticity outcome_adaptive --threats 1 --food-spawn-interval 20 --food-spawn-n 1 --output-dir test_results/cond_active_threat_adaptive

python baseline_passive_lower_bound.py --mode active --plasticity outcome_adaptive_signed --threats 1 --food-spawn-interval 20 --food-spawn-n 1 --output-dir test_results/cond_active_threat_adaptive_signed


---


python baseline_passive_lower_bound.py --mode active --plasticity none --threats 0 --food-spawn-interval 20 --food-spawn-n 1 --output-dir test_results/cond_active_threat_none_no_threat

python baseline_passive_lower_bound.py --mode active --plasticity outcome --threats 0 --food-spawn-interval 20 --food-spawn-n 1 --output-dir test_results/cond_active_threat_outcome_no_threat

python baseline_passive_lower_bound.py --mode active --plasticity outcome_adaptive --threats 0 --food-spawn-interval 20 --food-spawn-n 1 --output-dir test_results/cond_active_threat_adaptive_no_threat

python baseline_passive_lower_bound.py --mode active --plasticity outcome_adaptive_signed --threats 0 --food-spawn-interval 20 --food-spawn-n 1 --output-dir test_results/cond_active_threat_adaptive_signed_no_threat