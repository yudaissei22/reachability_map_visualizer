# reachability_map_visualizer

![Screenshot from 2024-12-12 10-23-17](https://github.com/user-attachments/assets/14e43ef1-2e8a-4624-92ff-d6a9a6c99127)

実行方法は、
'''
choreonoid "$(rospack find reachability_map_visualizer_ironman_upper)/config/jaxon_hand.cnoid"
'''

表示は、
'''
choreonoid "$(rospack find reachability_map_visualizer_ironman_upper)/config/visualize.cnoid"

'''


libCnoidReachabilityMapVisualizerSamplePlugin.soが2個追加されると、
jaxonでの探索領域に上書きされるので、
'''
catkin build reachability_map_visualizer_ironman_upper --no-deps --pre-clean
'''
をするとよい。
てか、しないとダメ。
