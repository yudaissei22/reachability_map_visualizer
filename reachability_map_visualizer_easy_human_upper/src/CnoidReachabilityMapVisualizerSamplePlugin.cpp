#include <cnoid/Plugin>
#include <cnoid/ItemManager>

#include <cnoid/Item>
#include <cnoid/LazyCaller>
#include <memory>

#include <choreonoid_viewer/choreonoid_viewer.h>

namespace reachability_map_visualizer_sample{
  void easy_human_upper();
  class jaxon_handItem : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<jaxon_handItem>("jaxon_handItem"); }
  protected:
    virtual void main() override{ easy_human_upper(); return;}
  };
  typedef cnoid::ref_ptr<jaxon_handItem> jaxon_handItemPtr;


  // --- viewerのためにtest
  // void visualize();
  // class visualizeItem : public choreonoid_viewer::ViewerBaseItem {
  // public:
  //   static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<visualizeItem>("visualizeItem"); }
  // protected:
  //   virtual void main() override{ visualize(); return;}
  // };
  // typedef cnoid::ref_ptr<visualizeItem> visualizeItemPtr;

  void view_4uu();
  class view4UUItem : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<view4UUItem>("view4UUItem"); }
  protected:
    virtual void main() override{ view_4uu(); return;}
  };
  typedef cnoid::ref_ptr<view4UUItem> view4UUItemPtr;
  
  // --- viewerのためにtest

  std::shared_ptr<choreonoid_viewer::Viewer> visualize();
  class visualizeItem : public cnoid::Item {
  public:
    visualizeItem()
      : showLater_([this](){
		     if (findRootItem() && !viewer_){
		       viewer_ = visualize();		       
		     }
		   }, cnoid::LazyCaller::NormalPriority)
      {
      }

    static void initializeClass(cnoid::ExtensionManager* ext){
      ext->itemManager().registerClass<visualizeItem>("visualizeItem");
    }

  protected:
    void onConnectedToRoot() override {
      showLater_();
    }

    void onDisconnectedFromRoot() override {
      showLater_.cancel();
      viewer_.reset();
    }
    
  private:
    std::shared_ptr<choreonoid_viewer::Viewer> viewer_;
    cnoid::LazyCaller showLater_;
		   
  };
  typedef cnoid::ref_ptr<visualizeItem> visualizeItemPtr;

  //  ---

  void solvability_bar();
  class solvability_barItem : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<solvability_barItem>("solvability_barItem"); }
  protected:
    virtual void main() override{ solvability_bar(); return;}
  };
  typedef cnoid::ref_ptr<solvability_barItem> solvability_barItemPtr;

  class ReachabilityMapVisualizerSamplePlugin : public cnoid::Plugin
  {
  public:
    ReachabilityMapVisualizerSamplePlugin() : Plugin("ReachabilityMapVisualizerSamplePlugin")
    {
      require("Body");
    }
    virtual bool initialize() override
    {
      jaxon_handItem::initializeClass(this);
      visualizeItem::initializeClass(this);
      view4UUItem::initializeClass(this);
      solvability_barItem::initializeClass(this);
      return true;
    }
  };
}

CNOID_IMPLEMENT_PLUGIN_ENTRY(reachability_map_visualizer_sample::ReachabilityMapVisualizerSamplePlugin)
