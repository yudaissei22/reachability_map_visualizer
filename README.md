# reachability_map_visualizer

![Screenshot from 2024-12-12 10-23-17](https://github.com/user-attachments/assets/14e43ef1-2e8a-4624-92ff-d6a9a6c99127)

IronmanとEasyHumanは、それぞれ専用の名前空間、共有ライブラリ、プラグイン登録名を使います。

| パッケージ | 共有ライブラリ | `.cnoid` の `plugin` |
| --- | --- | --- |
| `reachability_map_visualizer_ironman_upper` | `libCnoidIronmanReachabilityPlugin.so` | `IronmanReachability` |
| `reachability_map_visualizer_easy_human_upper` | `libCnoidEasyHumanReachabilityPlugin.so` | `EasyHumanReachability` |

ビルドはワークスペースのルートで実行します。変更を反映するには、ビルド後にChoreonoidを再起動してください。

```bash
catkin build reachability_map_visualizer_ironman_upper reachability_map_visualizer_easy_human_upper --no-deps
```

Ironmanの到達可能領域の計算:

```bash
choreonoid "$(rospack find reachability_map_visualizer_ironman_upper)/config/UronMan.cnoid"
```

Ironmanの計算済みマップの表示:

```bash
choreonoid "$(rospack find reachability_map_visualizer_ironman_upper)/config/visualize.cnoid"
```

EasyHumanの到達可能領域の計算:

```bash
choreonoid "$(rospack find reachability_map_visualizer_easy_human_upper)/config/easy_human_upper.cnoid"
```

EasyHumanの計算済みマップの表示:

```bash
choreonoid "$(rospack find reachability_map_visualizer_easy_human_upper)/config/visualize.cnoid"
```
