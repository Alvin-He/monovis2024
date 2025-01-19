lowering the image resolution passed to esitmator seems to speed things up a lot
but will consume gpu
also camera matrix will need to be transformed by each axis's repective scaling ratio


for boost 1.85 `cancel_on_conn_lost()` must be added to redis/detail/connection_base.hpp, around line 585

./include/boost/redis/detail/connection_base.hpp
```
            cancel_unwritten_requests();
         } break;
         case operation::run:
         {
            // Protects the code below from being called more than
            // once, see https://github.com/boostorg/redis/issues/181
            if (std::exchange(cancel_run_called_, true)) {
               cancel_on_conn_lost(); // <-------- add this
               return;
            }

            close();
            writer_timer_.cancel();
            receive_channel_.cancel();
            cancel_on_conn_lost();
         } break;
```

untill this issue gets resolved https://github.com/boostorg/redis/issues/211



for solvePnp, cv::SOLVEPNP_SQPNP is significantly faster 
so most of the performance pain points is on detection actually 



## Architecture

service based

EstimateApriltag: Estimates and returns all the corners in turns of frame positions

PoseTracker: Takes all the corners, solvepnp it into poses then combine all the poses into a global parameter