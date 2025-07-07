# cabot-people test

You can evalute tracking accuracy for cabot-navigation test case by following steps

## Set up test environment

Set up cabot-navigation test environment by following [the guide](https://github.com/CMU-cabot/cabot-navigation/blob/main/cabot_navigation2/test/README.md)

## Prepare test cases list YAML file

Check available test functions for your cabot site
```
./launch.sh -s -t -S <CABOT_SITE> -l
## example to list test functions
./launch.sh -s -t -S cabot_site_large_room -l
```

Select functions for tracking evaluation from available test functions, and create a YAML file in following format
```
tests:
  - site: <CABOT_SITE>
    module: <optional: test module>
    name: <test function>
```

Following is an example of YAML file
```
tests:
  - site: cabot_site_large_room
    func: test_category1_case1_move_towards_a_pedestrian
  - site: cabot_site_test_room
    module: tests-social
    func: test2_avoid_obstacle
```

## Evaluate tracking by batch test on the simulator

Run batch test for tracking evaluation
```
python3 ./test/run_test_batch.py -i <test cases YAML> -N <cabot-navigation directory> -O <log output directory>

## example to run batch test
python3 ./test/run_test_batch.py -i ./test/test-cases.yaml -N ../cabot-navigation -O ./test/test-cases-log
```

## Evaluate tracking by a test case on the simulator

Run a test case for tracking evaluation (results will be saved in the "eval_track_error" within the ROS log directory)
```
./test/run_test.sh -N <cabot-navigation directory> -S <cabot site name> -f <test name pattern> -O <log output directory>

## example to run a test case for cabot_site_large_room
./test/run_test.sh -N ../cabot-navigation -S cabot_site_large_room -f test_category1_case1_move_towards_a_pedestrian -O ./test/test-cases-log
```
