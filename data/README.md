# data/

Rosbags, extracted scans and datasets. Contents are git-ignored; only this note is tracked.
Record with `ros2 bag record` into `data/bags/<date>-<what>/`; derived arrays go to `data/datasets/`.
Every dataset is rebuilt from bags by a script in `ml/`, so nothing here is the source of truth.
