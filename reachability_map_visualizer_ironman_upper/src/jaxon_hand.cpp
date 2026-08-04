#include <reachability_map_visualizer/reachability_map_visualizer.h>
#include <choreonoid_viewer/choreonoid_viewer.h>
#include <cnoid/Body>
#include <cnoid/BodyLoader>
#include <iostream>
#include <ros/package.h> /* rosのpackageのpathを取得する方法です。 */

// ここにik_constraint2をつかって、関節角度の拘束を追加していく。
namespace reachability_map_visualizer_sample{
  /* namespaceは関数名の名前の衝突を防ぐもの */
  // この中に書いたものは、reachability_map_visualizer_sample::という名前空間に属しています。
  void jaxon_hand(){
    // voidなので、戻り値が無し。
    // jaxon_handは当然関数名です。
    std::shared_ptr<choreonoid_viewer::Viewer> viewer = std::make_shared<choreonoid_viewer::Viewer>();
    /*  std::shared_ptr<T>は、共有所有ポインタ。メモリ管理を自動でしてくれる、便利なポインタです。これは、viewerという変数の型を指定するもの。
	viewerという変数に、Viewerというオブジェクトを作っていれています。
	make_chared<>は、shared_ptrを安全に作る為の定番の書き方です。
	恐らく、choreonoid_viewerという名前空間の中にある、Viewerというクラスです。
	これは、ようするに、choreonoid_viewerという型のオブジェクト（Viewer）を1個作って、それのmake_sharedで共有できるようにポインタ作って、それをviewerという変数に入れるんだ。
     */
    std::shared_ptr<reachability_map_visualizer::ReachabilityMapParam> param = std::make_shared<reachability_map_visualizer::ReachabilityMapParam>();
    /*
      これも、ReachabilityMapParamという変数を、reachability_map_visualizerという名前空間において、ReachabilityMapParamというクラスを作って、そのオブジェクトのポンタをparamに入れる。
     */
    
    cnoid::BodyLoader bodyLoader;
    // bodyloaderは、cnoid::BodyLoader型の変数bodyLoaderを作っていて、これは、ロボットモデルを読むためのオブジェクト。
    param->robot = bodyLoader.load(ros::package::getPath("ironman_models") + "/IronMan_for_choreonoid_ver2.wrl");
    /*
      param->robotは、paramというオブジェクトがさすrobotメンバ。
      さかのぼって実装を見てみると、cnoid::BodyPtr robot;というものがある。これのことか！
     */
    if(!(param->robot)) std::cerr << "!robot" << std::endl;
    /*
      これはよくある例外処理だね、無かったら、!つけてTrueにして、エラー分を出す
    */
    
    param->robot->rootLink()->p() = cnoid::Vector3(0,0,0.0);
    /*
      paramのrobotの、rootLink()のp()なので、位置ベクトル、これを0,0,0に固定する。
      choreonoidなので、cnoid::Vector3()という型になっています。
     */
    param->robot->rootLink()->v().setZero();
    /*
      これもルートリンクの並進速度をゼロにする。
    */
    param->robot->rootLink()->R() = cnoid::Matrix3::Identity();
    /*
      ルートリンクの、R()なので、回転行列を、cnoid::Matrix3::Identity()は、単位行列なので、回転無しにする。
     */
    param->robot->rootLink()->w().setZero();
    /*
      ルートリンクの姿勢を回転無しに設定する。
     */
    std::vector<double> reset_manip_pose{
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
    };

    for(int j=0; j < param->robot->numJoints(); ++j){
      param->robot->joint(j)->q() = reset_manip_pose[j];
    }
    /*
      これは、robotのjoint(j)の関節角度q()を、reset_manip_pose[j]を入れるので、全関節に対して、reset_manip_poseの値をぶち込む作業。
     */
    param->robot->calcForwardKinematics();
    /*
      準運動学を計算する。初期姿勢を反映して、ロボット全体の状態を更新している。
      各関節角度が与えられたときに、EEの位置姿勢や各リンクの位置姿勢を求めている。
     */
    param->robot->calcCenterOfMass();
    
    
    // joint limit　以下は関節制限の設定に行く。

    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints;
    /*
      IKConstraint型のshared_ptrを並べるvectorを作っている。
      constraintsが制約のリストで。

      恐らく、どうやら、IK制約をまとめて保存する配列を用意しているそう。
     */



    for(int i=0;i<param->robot->numJoints();i++){
      std::shared_ptr<ik_constraint2::JointLimitConstraint> constraint = std::make_shared<ik_constraint2::JointLimitConstraint>();
      /*
	関節可動域制限のオブジェクト。
	関節1つ分の制約を作っているんだって。
       */
      constraint->joint() = param->robot->joint(i);
      /*
	作った制約に対して、「どの関節に対する制約にするか？を決めているもの」
      */
      constraints.push_back(constraint);
      /*
	push_back ... 配列の末尾に追加する関数
       */
    }

    /* ここにシンプルに追加したらいいんでないのかな？ 分からないので検証とかします。
       これはUniversal jointのパラレルリンクによる関節角度拘束を表すものです。
     */
    // param->robot->joint(0)->q() = -param->robot->joint(3)->q();
    // param->robot->joint(1)->q() = -param->robot->joint(2)->q();
    // これだとik解くときに、この拘束は反映されないので意味なし

    {
      std::shared_ptr<ik_constraint2::JointAngleConstraint_4UU> constraint = std::make_shared<ik_constraint2::JointAngleConstraint_4UU>();
      constraint->A_joint() = param->robot->joint("B4_base");
      constraint->B_joint() = param->robot->joint("SHOULDER_PITCH");
      constraints.push_back(constraint);
    }

    {
      std::shared_ptr<ik_constraint2::JointAngleConstraint_4UU> constraint = std::make_shared<ik_constraint2::JointAngleConstraint_4UU>();
      constraint->A_joint() = param->robot->joint("B4_UU");
      constraint->B_joint() = param->robot->joint("B4_END_YAW");
      constraints.push_back(constraint);
    }
      
    /* ik変数を登録? */
    param->constraints.push_back(constraints);
    /*
      これは、param->constraintsに、今作った制約群を追加していうｒ。
      constrainsは、制約のセットみたいな扱いですねぇ。
     */

    // param->variables.push_back(param->robot->joint("B4_BASE"));
    param->variables.push_back(param->robot->joint("B4_base")); // yaw     
    param->variables.push_back(param->robot->joint("B4_UU")); // roll
    param->variables.push_back(param->robot->joint("B4_END_YAW")); // roll
    param->variables.push_back(param->robot->joint("SHOULDER_PITCH")); // yaw
    param->variables.push_back(param->robot->joint("SHOULDER_ROLL"));
    param->variables.push_back(param->robot->joint("SHOULDER_YAW"));
    param->variables.push_back(param->robot->joint("ELBOW_PITCH"));
    param->variables.push_back(param->robot->joint("WRIST_YAW"));

    /*
      variablesは、動かしてよい変数の一覧のこと。
      ここでは、名前で関数を取得していますが。。。名前は何を参照しているのかな？
      Q[恐らく、cnoid::BodyPtr robot;としてrobotが定義されているので、cnoidの方でwrlから読み取るという何かしらの処理が行われているのかな？]
     */
    reachability_map_visualizer::EndEffector ee;
    /*
      Endeffector型の変数eeを作成。
      手先の情報を表す構造体と考えてよい層。
    */
    ee.parent = param->robot->link("WRIST_YAW");
    /* 
       なんでか、EEの親リンクを設定していて、それがwrist_yawなんだとか。
       Q. この親リンクを設定する必要性ってなんでなんだろうか？
     */
    param->endEffectors.push_back(ee);
    /* 
       そもそも、endEffectorsという変数が、reahcabivility_map_visualizerの、
       std::vector<EndEffector> endEffectors;
       で定義されているのだが、これはmapptingするから、EEのいろんないち姿勢を表したものということかな？
       到達可能性判定の対象となる手先を登録する
     */
    param->posResolution = 0.05;
    /*
      poreResolutionは、位置の分解能。0.05は恐らく、5cm刻みということ
     */
    param->pikParam.maxIteration = 30;
    /*
      maxIterationは、反復計算の最大回数。
      つまり、IKを解くときの試行回数上限です。
     */
    param->testPerGrid = 50;
    /*
      testPerGridは、1グリッドあたり何回テストするのか？ということ？
      といわれても、あまりピンと来ないのだが。
     */
    param->origin = cnoid::Vector3(0.25, -0.15, 0.0);
    /*
      originは何？を表しているのか？だが、恐らくマップ領域の原点くさくて、それを設定している。
     */
    param->size = cnoid::Vector3(0.6, 0.6, 0.6);
    /*
      調べる空間サイズを設定していて、x,y,z方向に60cmずつです。
      cnoid::Vector6 weight; //ランダムに選んだ姿勢座標系らしいが、どういうことなのかな？あまり良く分からない。
      このあたりはIK詳しくないと理論的なことが分かってないと、理解できなそうではあるなぁ。
     */
    param->weight[5] = 0.0;
    std::shared_ptr<reachability_map_visualizer::ReachabilityMap> map = std::make_shared<reachability_map_visualizer::ReachabilityMap>();
    /*
      到達可能領域を示すオブジェクトmapを作成していきます。
      <reachability_map_visualizerという名前空間において、::でクラス名で、クラスメイは、reachabilityMap>
      それをstd::make_sharedでポインタを共有できるようにしてい？て、それをmapという変数に入れている。
     */
    // reachability_map_visualizer::createMap(param, map);
    reachability_map_visualizer::createMap(param, map, 4);
    /* 
     createmapという関数があって、もうそれを呼び出しているね。
     んで、この関数はどこで定義されているのか？というと、
     みたけど、src/hoge.cppにあって、for分を回してしっかり計算してくれていそうだった。
     */

    reachability_map_visualizer::writeMap(ros::package::getPath("reachability_map_visualizer_ironman_upper") + "/config/ironman_upper_hand.yaml",map);
    /*
      これはmapを書いて、yamlファイルに書き出す、みたら分かりそうですね。
     */
    viewer->objects(param->robot);
    /*
      これは恐らくchoreonoid viewerみたいに、viewerにrobot自体を登録して表示している。
     */
    reachability_map_visualizer::visualizeMap(map, viewer);
    viewer->drawObjects();
  }
}
