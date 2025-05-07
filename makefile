.PHONY: install run_cam ros_pipeline run_hosted

install:
	$(PIP) install -r requirements.txt

run_cam:
	PYTHONPATH=$(PWD) -- python -m scripts.open_cam

ros_pipeline:
	source /opt/ros/humble/setup.bash && \
	source install/setup.bash && \
	ros2 launch launch/asl_pipeline.launch.py

run_hosted:
	PYTHONPATH=$(PWD) dotenv run -- python -m scripts.remote_asl_cam
	