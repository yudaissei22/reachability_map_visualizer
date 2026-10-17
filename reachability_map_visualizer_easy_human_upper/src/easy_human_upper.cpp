#include <reachability_map_visualizer/reachability_map_visualizer.h>
#include <choreonoid_viewer/choreonoid_viewer.h>
#include <cnoid/Body>
#include <cnoid/BodyLoader>
#include <iostream>
#include <ros/package.h> /* rosのpackageのpathを取得する方法です。 */
#include <ik_constraint2_bullet/BulletCollisionConstraint.h>
#include <memory>
#include <string>
#include <utility>
#include <vector>


// ここにik_constraint2をつかって、関節角度の拘束を追加していく。
namespace reachability_map_visualizer_easy_human_upper{
  /* namespaceは関数名の名前の衝突を防ぐもの */
  // この中に書いたものは、reachability_map_visualizer_easy_human_upper::という名前空間に属しています。
  void easy_human_upper(){
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
    param->robot = bodyLoader.load(ros::package::getPath("UronMan_models") + "/EasyHuman_Cylinder.wrl");
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
					 0.0, 0.0, 0.0, 0.0,
					 0.0, 0.0, 0.0, 0.0,					 
					 0.0, 0.0, 0.0,
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
      constraint->A_joint() = param->robot->joint("U1_CORE");
      constraint->B_joint() = param->robot->joint("SHOULDER_FIRST");
      constraints.push_back(constraint);
    }

    {
      std::shared_ptr<ik_constraint2::JointAngleConstraint_4UU> constraint = std::make_shared<ik_constraint2::JointAngleConstraint_4UU>();
      constraint->A_joint() = param->robot->joint("UU");
      constraint->B_joint() = param->robot->joint("U2_CORE");
      constraints.push_back(constraint);
    }

    /* ikのを登録 */
    param->constraints.push_back(constraints);

    
      
    /* collisionの制約 */
    // 直接の親子関係にあるリンクは対象から除外する。
    
    const std::vector<std::pair<std::string, std::string>> collisionPairs = {
									     
    // 胴体は上腕以降との衝突を判定する。
    // 肩甲骨側の U1_CORE / UU / U2_CORE / SHOULDER_FIRST は対象外。
    // SHOULDER_PITCH は形状なし。
    {"ROOT", "SHOULDER_ROLL"},
    {"ROOT", "SHOULDER_TO_ELBOW"},
    {"ROOT", "ELBOW_UPPER"},
    {"ROOT", "ELBOW_LOWER"},
    {"ROOT", "WRIST"},
    {"ROOT", "HAND"},

    // U1_CORE
    {"U1_CORE", "U2_CORE"},
    {"U1_CORE", "SHOULDER_FIRST"},
    // {"U1_CORE", "SHOULDER_PITCH"},
    {"U1_CORE", "SHOULDER_ROLL"},
    {"U1_CORE", "SHOULDER_TO_ELBOW"},
    {"U1_CORE", "ELBOW_UPPER"},
    {"U1_CORE", "ELBOW_LOWER"},
    {"U1_CORE", "WRIST"},
    {"U1_CORE", "HAND"},

    // UU
    // {"UU", "SHOULDER_PITCH"},
    {"UU", "SHOULDER_ROLL"},
    {"UU", "SHOULDER_TO_ELBOW"},
    {"UU", "ELBOW_UPPER"},
    {"UU", "ELBOW_LOWER"},
    {"UU", "WRIST"},
    {"UU", "HAND"},

    // U2_CORE
    // {"U2_CORE", "SHOULDER_PITCH"},
    {"U2_CORE", "SHOULDER_ROLL"},
    {"U2_CORE", "SHOULDER_TO_ELBOW"},
    {"U2_CORE", "ELBOW_UPPER"},
    {"U2_CORE", "ELBOW_LOWER"},
    {"U2_CORE", "WRIST"},
    {"U2_CORE", "HAND"},

    // SHOULDER_FIRST
    {"SHOULDER_FIRST", "SHOULDER_TO_ELBOW"},
    {"SHOULDER_FIRST", "ELBOW_UPPER"},
    {"SHOULDER_FIRST", "ELBOW_LOWER"},
    {"SHOULDER_FIRST", "WRIST"},
    {"SHOULDER_FIRST", "HAND"},

    // SHOULDER_PITCH（形状なし）
    // {"SHOULDER_PITCH", "SHOULDER_TO_ELBOW"},
    // {"SHOULDER_PITCH", "ELBOW_UPPER"},
    // {"SHOULDER_PITCH", "ELBOW_LOWER"},
    // {"SHOULDER_PITCH", "WRIST"},
    // {"SHOULDER_PITCH", "HAND"},

    // SHOULDER_ROLL
    {"SHOULDER_ROLL", "ELBOW_LOWER"},
    {"SHOULDER_ROLL", "ELBOW_UPPER"},    
    {"SHOULDER_ROLL", "WRIST"},
    {"SHOULDER_ROLL", "HAND"},

    // SHOULDER_TO_ELBOW
    {"SHOULDER_TO_ELBOW", "ELBOW_LOWER"},
    {"SHOULDER_TO_ELBOW", "WRIST"},
    {"SHOULDER_TO_ELBOW", "HAND"},

    // ELBOW_UPPER
    {"ELBOW_UPPER", "WRIST"},
    {"ELBOW_UPPER", "HAND"},

    // ELBOW_LOWER
									     
    };

    for (const auto& pair : collisionPairs) {
      cnoid::LinkPtr linkA = param->robot->link(pair.first);
      cnoid::LinkPtr linkB = param->robot->link(pair.second);      

    // リンク名の間違いを
    if (!linkA || !linkB){
      std::cerr << "Collision link not found: "
		<< pair.first <<  " / " << pair.second << std::endl;
      return;
    }

    // 衝突用形状が登録されることを確認
    if (!linkA->collisionShape() || !linkB->collisionShape()){
        std::cerr << "Collision shape missing: "
                  << pair.first << " / " << pair.second << std::endl;
        return;      
    }

    auto collision =
      std::make_shared<ik_constraint2_bullet::BulletCollisionConstraint>();

    collision->A_link() = linkA;
    collision->B_link() = linkB;

    // 距離の単位は m。
    collision->tolerance() = 0.005;     // 要求する離隔距離：5 mm
    collision->precision() = 0.0001;    // 判定の許容誤差：0.1 mm
    collision->ignoreDistance() = 0.10; // 10 cm以上離れたら制約を外す

    // 各リンク全体を1つの凸包として近似する。
    collision->useSingleMesh() = true;

    // 並列計算に入る前に衝突用モデルを生成・キャッシュする。
    // この時点では、既存処理で順運動学が計算されている。
    collision->updateBounds();

    if (collision->A_bulletModel().empty() ||
        collision->B_bulletModel().empty()) {
        std::cerr << "Failed to create collision model: "
                  << pair.first << " / " << pair.second << std::endl;
        return;
    }
    constraints.push_back(collision);
  }

    // 関節制約と衝突制約をまとめて登録する。
    param->constraints.push_back(constraints);

    /*
      これは、param->constraintsに、今作った制約群を追加していうｒ。
      constrainsは、制約のセットみたいな扱いですねぇ。
     */

    // param->variables.push_back(param->robot->joint("B4_BASE"));
    param->variables.push_back(param->robot->joint("U1_CORE")); // yaw     
    param->variables.push_back(param->robot->joint("UU")); // roll
    param->variables.push_back(param->robot->joint("U2_CORE")); // roll
    param->variables.push_back(param->robot->joint("SHOULDER_FIRST")); // yaw        
    param->variables.push_back(param->robot->joint("SHOULDER_ROLL")); // yaw    
    param->variables.push_back(param->robot->joint("SHOULDER_PITCH")); // yaw    
    param->variables.push_back(param->robot->joint("SHOULDER_TO_ELBOW"));
    param->variables.push_back(param->robot->joint("ELBOW_UPPER"));
    param->variables.push_back(param->robot->joint("ELBOW_LOWER"));
    param->variables.push_back(param->robot->joint("WRIST"));
    param->variables.push_back(param->robot->joint("HAND"));

    /*
      variablesは、動かしてよい変数の一覧のこと。n
      ここでは、名前で関数を取得していますが。。。名前は何を参照しているのかな？
      Q[恐らく、cnoid::BodyPtr robot;としてrobotが定義されているので、cnoidの方でwrlから読み取るという何かしらの処理が行われているのかな？]
     */
    reachability_map_visualizer::EndEffector ee;
    /*
      Endeffector型の変数eeを作成。
      手先の情報を表す構造体と考えてよい層。
    */
    ee.parent = param->robot->link("HAND");
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
    // 元のやつ
    // param->origin = cnoid::Vector3(0.25, -0.15, 0.0);
    param->origin = cnoid::Vector3(-0.010, 0.350, 0.0);
    
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

    reachability_map_visualizer::writeMap(ros::package::getPath("reachability_map_visualizer_easy_human_upper") + "/config/easy_human_upper.yaml",map);
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
