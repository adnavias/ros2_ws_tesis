// Plugin de sistema para Gazebo Fortress (ignition-gazebo6) que impone la
// configuración articular inicial del brazo UNA sola vez. No aplica fuerzas
// ni cierra ningún lazo: tras el primer paso el sistema evoluciona libremente
// (respuesta sin control).
// Autor: Adán Medina Covarrubias

#include <map>
#include <memory>
#include <string>

#include <ignition/common/Console.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/components/JointPositionReset.hh>
#include <ignition/gazebo/components/JointVelocityReset.hh>
#include <ignition/plugin/Register.hh>
#include <sdf/Element.hh>

namespace mi_rover
{

namespace gzs = ignition::gazebo;

class InitialJointPositions final
  : public gzs::System,
    public gzs::ISystemConfigure,
    public gzs::ISystemPreUpdate
{
public:
  void Configure(
    const gzs::Entity & entity,
    const std::shared_ptr<const sdf::Element> & sdf,
    gzs::EntityComponentManager & /*ecm*/,
    gzs::EventManager & /*event_mgr*/) override
  {
    m_model = gzs::Model(entity);

    // Lectura de <joint name="...">valor</joint> desde el bloque <plugin>.
    const sdf::ElementPtr root = sdf->Clone();
    for (sdf::ElementPtr elem = root->FindElement("joint"); elem != nullptr;
      elem = elem->GetNextElement("joint"))
    {
      m_targets[elem->Get<std::string>("name")] = elem->Get<double>();
    }

    if (m_targets.empty())
    {
      ignwarn << "[InitialJointPositions] Sin articulaciones configuradas.\n";
    }
  }

  void PreUpdate(
    const gzs::UpdateInfo & /*info*/,
    gzs::EntityComponentManager & ecm) override
  {
    if (m_applied)
    {
      return;
    }

    for (const auto & [name, value] : m_targets)
    {
      const gzs::Entity joint = m_model.JointByName(ecm, name);
      if (joint == gzs::kNullEntity)
      {
        ignerr << "[InitialJointPositions] No existe la articulación '"
               << name << "'.\n";
        continue;
      }
      setReset<gzs::components::JointPositionReset>(ecm, joint, value);
      setReset<gzs::components::JointVelocityReset>(ecm, joint, 0.0);
      ignmsg << "[InitialJointPositions] " << name << " = " << value << "\n";
    }
    m_applied = true;
  }

private:
  // Crea o actualiza el componente de reset; Physics lo consume y lo elimina
  // en su Update, por eso el reset ocurre exactamente una vez.
  template <typename ResetComponent>
  static void setReset(
    gzs::EntityComponentManager & ecm, const gzs::Entity joint,
    const double value)
  {
    auto * comp = ecm.Component<ResetComponent>(joint);
    if (comp == nullptr)
    {
      ecm.CreateComponent(joint, ResetComponent({value}));
    }
    else
    {
      comp->Data() = {value};
    }
  }

  gzs::Model m_model{gzs::kNullEntity};
  std::map<std::string, double> m_targets;
  bool m_applied{false};
};

}  // namespace mi_rover

IGNITION_ADD_PLUGIN(
  mi_rover::InitialJointPositions,
  ignition::gazebo::System,
  mi_rover::InitialJointPositions::ISystemConfigure,
  mi_rover::InitialJointPositions::ISystemPreUpdate)
