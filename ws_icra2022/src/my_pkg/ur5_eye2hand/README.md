# 总结

* 0827晚上转到eye to hand
* 自适应的一个小trick: 让\hat\theta_k中的r31 r32 r33三项尽量准确（初值近+更新率小），控制效果就好

* 最终用到的有
  * keyboard_adaptive0915.py 读取hololens2_adaptive/0914-final-2/log_d.npy跑出深度
  * hololens2_adaptive.py 数据在hololens2_adaptive/0914-final-2
  * hololens2_adaptive_circle.py 数据在hololens2_adaptive_circle
  * keyboard_adaptive_simplified.py原本认为\theta_z需要用13个元素，后来经翔哥提醒发现只需要4个。

