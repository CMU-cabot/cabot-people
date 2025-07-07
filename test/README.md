# cabot-people test

You can evalute tracking accuracy for cabot-navigation test case by following steps.

## Evaluate tracking by batch test on the simulator

Run batch test for tracking evaluation
```
python3 ./test/run_test_batch.py -i <test cases YAML> -N <cabot-navigation directory>

## example to run batch test
python3 ./test/run_test_batch.py -i ./test/test-cases.yaml -N ../cabot-navigation 
```

## Evaluate tracking by a test case on the simulator

Run a test case for tracking evaluation (results will be saved in the "eval_track_error" within the ROS log directory)
```
./test/run_test.sh -N <cabot-navigation directory> -S <cabot site name> -f <test name pattern>

## example to run a test case for cabot_site_large_room
./test/run_test.sh -N ../cabot-navigation -S cabot_site_large_room -f test_category1_case1_move_towards_a_pedestrian
```
