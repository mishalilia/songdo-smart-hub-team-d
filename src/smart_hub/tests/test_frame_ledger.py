# Author: Mikhail Li (12236295)
"""Frame accounting of the camera path: nothing received is left unreported."""
from smart_hub.passenger_detector import FrameLedger


def test_every_received_frame_is_processed_dropped_or_expired():
    ledger = FrameLedger()
    ledger.offer()                       # frame 1 waits
    ledger.offer()                       # frame 2 replaces it: one drop
    assert ledger.take(age_sec=0.10, max_age_sec=1.0)
    ledger.done(latency_ms=310.0)
    ledger.offer()                       # frame 3 waited too long
    assert not ledger.take(age_sec=1.4, max_age_sec=1.0)
    ledger.offer()
    assert ledger.take(age_sec=0.05, max_age_sec=1.0)
    ledger.done(latency_ms=250.0)
    assert (ledger.received, ledger.processed, ledger.dropped, ledger.expired) == (4, 2, 1, 1)
    assert ledger.received == ledger.processed + ledger.dropped + ledger.expired
    assert ledger.report() == 'received=4 processed=2 dropped=1 expired=1 ' \
        'acquisition_to_update_ms_p95=310.0 p99=310.0 n=2'


def test_report_without_processed_frames_has_no_latency():
    ledger = FrameLedger()
    ledger.offer()
    assert ledger.report() == 'received=1 processed=0 dropped=0 expired=0'
