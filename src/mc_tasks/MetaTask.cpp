#include <mc_tasks/MetaTask.h>

#include <mc_rtc/logging.h>

namespace mc_tasks
{

double extraStiffness(double error, double extraStiffness)
{
  return exp(-error)*extraStiffness;
}

void MetaTask::load(mc_solver::QPSolver & solver,
                    const mc_rtc::Configuration & config)
{
  if(config.has("dimWeight"))
  {
    Eigen::VectorXd dimW = config("dimWeight");
    if(dimW.size() != dimWeight().size())
    {
      LOG_ERROR_AND_THROW(std::runtime_error, "Stored dimWeight has the wrong dimension (is " << dimW.size() << " should be " << dimWeight().size())
    }
  }
  if(config.has("activeJoints"))
  {
    selectActiveJoints(solver, config("activeJoints"));
  }
  if(config.has("unactiveJoints"))
  {
    selectUnactiveJoints(solver, config("unactiveJoints"));
  }
  if(config.has("name"))
  {
    name(config("name"));
  }
}

void MetaTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  gui.addElement(
    mc_rtc::gui::Element<std::string>{
      {"Tasks", name_, "type"},
      [this]() { return this->type_; }
    }
  );
  gui.addElement(
    mc_rtc::gui::Element<Eigen::VectorXd>{
      {"Tasks", name_, "eval"},
      [this]() { return this->eval(); }
    },
    mc_rtc::gui::Label(true)
  );
  gui.addElement(
    mc_rtc::gui::Element<Eigen::VectorXd>{
      {"Tasks", name_, "speed"},
      [this]() { return this->speed(); }
    },
    mc_rtc::gui::Label(true)
  );
}

void MetaTask::removeFromGUI(mc_rtc::gui::StateBuilder & gui)
{
  gui.removeCategory({"Tasks", name_});
}

}
