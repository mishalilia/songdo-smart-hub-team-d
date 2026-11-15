# Dispatch threshold ablation (S2)

Matrix A20261115-061707, commit cedb8c803ea2bac247ca98708d5981337ce6253b. Seeds 201-220 (not used by any earlier matrix or by the choice of q). Reactive: observed queue >= dispatch_high, dispatch_low = dispatch_high - 2. Predictive: installed configuration (forecast + validation residual quantile 0.8, observed floor at 6).

| Policy | Episodes passed | Boarded | All-passenger wait, lower bound, mean (s) | Boarded wait mean (s) | Completion |
|---|---|---|---|---|---|
| predictive | 20 | 237 | 116.7 | 105.7 | 0.566 |
| reactive-6 | 20 | 237 | 123.9 | 121.7 | 0.570 |
| reactive-5 | 20 | 244 | 115.6 | 106.9 | 0.574 |
| reactive-4 | 19 | 244 | 111.2 | 102.3 | 0.585 |

Paired difference per seed, predictive - reactive (and lower thresholds against reactive-6); 95 % interval from 10000 bootstrap resamples of whole episodes. Lead = first dispatch of the second policy minus the first.

| Comparison | Wait | Pairs | Identical arrivals | Mean delta (s) | 95 % interval (s) | First lower | Median lead (s) |
|---|---|---|---|---|---|---|---|
| predictive vs reactive-6 | all passengers | 20 | 20 | -7.2 | -10.1 .. -4.5 | 18 |  |
| predictive vs reactive-6 | boarded | 20 | 20 | -16.0 | -21.7 .. -10.4 | 18 | +18.2 (15 earlier, 0 later by >= 1 s) |
| predictive vs reactive-5 | all passengers | 20 | 20 | +1.1 | -1.0 .. +3.1 | 8 |  |
| predictive vs reactive-5 | boarded | 20 | 20 | -1.2 | -4.7 .. +1.8 | 10 | +1.1 (10 earlier, 7 later by >= 1 s) |
| predictive vs reactive-4 | all passengers | 19 | 19 | +5.2 | +3.1 .. +7.4 | 1 |  |
| predictive vs reactive-4 | boarded | 19 | 19 | +3.4 | +0.7 .. +5.9 | 3 | -2.7 (0 earlier, 19 later by >= 1 s) |
| reactive-5 vs reactive-6 | all passengers | 20 | 20 | -8.3 | -11.3 .. -5.8 | 20 |  |
| reactive-5 vs reactive-6 | boarded | 20 | 20 | -14.8 | -19.7 .. -10.2 | 20 | +16.0 (20 earlier, 0 later by >= 1 s) |
| reactive-4 vs reactive-6 | all passengers | 19 | 19 | -12.5 | -15.9 .. -9.2 | 19 |  |
| reactive-4 vs reactive-6 | boarded | 19 | 19 | -19.3 | -24.5 .. -14.3 | 19 | +23.6 (19 earlier, 0 later by >= 1 s) |
