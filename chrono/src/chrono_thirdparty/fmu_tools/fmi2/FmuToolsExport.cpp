#include <regex>
#include <fstream>

#include "fmi2/FmuToolsExport.h"
#include "FmuToolsRuntimeLinking.h"

#include "rapidxml/rapidxml_ext.hpp"

const std::unordered_set<UnitDefinitionType, UnitDefinitionType::Hash> common_unitdefinitions = {
    UD_kg,  UD_m,    UD_s,     UD_A,      UD_K, UD_mol, UD_cd, UD_rad,  //
    UD_m_s, UD_m_s2, UD_rad_s, UD_rad_s2,                               //
    UD_N,   UD_Nm,   UD_N_m2                                            //
};

bool is_pointer_variant(const FmuVariableBindType& myVariant) {
    return varns::visit([](auto&& arg) -> bool { return std::is_pointer_v<std::decay_t<decltype(arg)>>; }, myVariant);
}

void createModelDescription(const std::string& path, FmuType fmu_type) {
    fmi2Type fmi2_type = fmi2Type::fmi2CoSimulation;
    switch (fmu_type) {
        case FmuType::COSIMULATION:
            fmi2_type = fmi2Type::fmi2CoSimulation;
            break;
        case FmuType::MODEL_EXCHANGE:
            fmi2_type = fmi2Type::fmi2ModelExchange;
            break;
    }

    fmi2CallbackFunctions callfun = {LoggingUtilities::logger_default, calloc, free, nullptr, nullptr};
    FmuComponentBase* fmu = fmi2Instantiate_getPointer("", fmi2_type, FMU_GUID,
                                                       ("file:///" + GetLibraryLocation() + "/../../resources").c_str(),
                                                       &callfun, fmi2False, fmi2False);
    fmu->ExportModelDescription(path);
    delete fmu;
}

// =============================================================================

FmuVariableExport::FmuVariableExport(const VarbindType& varbind,
                                     const std::string& _name,
                                     FmuVariable::Type _type,
                                     CausalityType _causality,
                                     VariabilityType _variability,
                                     InitialType _initial)
    : FmuVariable(_name, _type, _causality, _variability, _initial) {
    // From FMI Reference
    // If initial = 'exact' or 'approx', or causality = 'input',       a start value MUST be provided.
    // If initial = 'calculated',        or causality = 'independent', a start value CANNOT be provided.
    if (initial == InitialType::calculated || causality == CausalityType::independent) {
        allowed_start = false;
        required_start = false;
    }

    if (initial == InitialType::exact || initial == InitialType::approx || causality == CausalityType::input) {
        allowed_start = true;
        required_start = true;
    }

    this->varbind = varbind;
}

FmuVariableExport::FmuVariableExport(const FmuVariableExport& other) : FmuVariable(other) {
    varbind = other.varbind;
    start = other.start;
    allowed_start = other.allowed_start;
    required_start = other.required_start;
}

FmuVariableExport& FmuVariableExport::operator=(const FmuVariableExport& other) {
    if (this == &other) {
        return *this;  // Self-assignment guard
    }

    FmuVariable::operator=(other);

    varbind = other.varbind;
    start = other.start;
    allowed_start = other.allowed_start;
    required_start = other.required_start;

    return *this;
}

void FmuVariableExport::SetValue(const fmi2String& val) const {
    if (is_pointer_variant(this->varbind))
        *varns::get<std::string*>(this->varbind) = std::string(val);
    else
        varns::get<FunGetSet<std::string>>(this->varbind).second(std::string(val));
}

void FmuVariableExport::GetValue(fmi2String* varptr) const {
    *varptr = is_pointer_variant(this->varbind) ? varns::get<std::string*>(this->varbind)->c_str()
                                                : varns::get<FunGetSet<std::string>>(this->varbind).first().c_str();
}

void FmuVariableExport::SetStartVal(fmi2String startval) {
    if (allowed_start)
        has_start = true;
    else
        return;
    this->start = std::string(startval);
}

void FmuVariableExport::ExposeCurrentValueAsStart() {
    if (required_start) {
        varns::visit([this](auto&& arg) { this->setStartFromVar(arg); }, this->varbind);

        // if (is_pointer_variant(this->varbind)){
        //     // check if string TODO: check if this check is needed .-)
        //     if (varns::holds_alternative<fmi2String*>(this->varbind)) {
        //         // TODO
        //         varns::visit([this](auto& arg){ return this->start = arg; }, this->varbind);
        //     }
        //     else{
        //         varns::visit([this](auto& arg){ return this->SetStartVal(*arg); }, this->varbind);
        //     }
        // }
        // else{

        //}
    }
}

std::string FmuVariableExport::GetStartVal_toString() const {
    // TODO: C++17 would allow the overload operator in lambda
    // std::string start_string;
    // varns::visit([&start_string](auto&& arg) -> std::string {return start_string = std::to_string(*start_ptr)});

    if (const fmi2Real* start_ptr = varns::get_if<fmi2Real>(&this->start))
        return std::to_string(*start_ptr);
    if (const fmi2Integer* start_ptr = varns::get_if<fmi2Integer>(&this->start))
        return std::to_string(*start_ptr);
    if (const fmi2Boolean* start_ptr = varns::get_if<fmi2Boolean>(&this->start))
        return std::to_string(*start_ptr);
    if (const std::string* start_ptr = varns::get_if<std::string>(&this->start))
        return *start_ptr;
    return "";
}

// =============================================================================

FmuComponentBase::FmuComponentBase(fmi2String instanceName,
                                   fmi2Type fmuType,
                                   fmi2String fmuGUID,
                                   fmi2String fmuResourceLocation,
                                   const fmi2CallbackFunctions* functions,
                                   fmi2Boolean visible,
                                   fmi2Boolean loggingOn,
                                   const std::unordered_map<std::string, bool>& logCategories_init,
                                   const std::unordered_set<std::string>& logCategories_debug_init)
    : m_instanceName(instanceName),
      m_callbackFunctions(*functions),
      m_fmuGUID(FMU_GUID),
      m_visible(visible == fmi2True ? true : false),
      m_debug_logging_enabled(loggingOn == fmi2True ? true : false),
      m_modelIdentifier(FMU_MODEL_IDENTIFIER),
      m_fmuMachineState(FmuMachineStateType::instantiated),
      m_logCategories_enabled(logCategories_init),
      m_logCategories_debug(logCategories_debug_init) {
    m_unitDefinitions["1"] = UnitDefinitionType("1");  // guarantee the existence of the default unit
    m_unitDefinitions[""] = UnitDefinitionType("");    // guarantee the existence of the unassigned unit

    AddFmuVariable(&m_time, "time", FmuVariable::Type::Real, "s", "time");

    //// Parse URL according to https://datatracker.ietf.org/doc/html/rfc3986
    std::string m_resources_location_str = std::string(fmuResourceLocation);

    std::regex url_patternA("^(\\w+):\\/\\/[^\\/]*\\/([^#\\?]+)");
    std::regex url_patternB("^(\\w+):\\/([^\\/][^#\\?]+)");

    std::smatch url_matches;
    std::string url_scheme;
    if ((std::regex_search(m_resources_location_str, url_matches, url_patternA) ||
         std::regex_search(m_resources_location_str, url_matches, url_patternB)) &&
        url_matches.size() >= 3) {
        if (url_matches[1].compare("file") != 0) {
            sendToLog("Bad URL scheme: " + url_matches[1].str() + ". Trying to continue.\n", fmi2Status::fmi2Warning,
                      "logStatusWarning");
        }
        m_resources_location = std::string(url_matches[2]) + "/";
    } else {
        // TODO: rollback?
        sendToLog("Cannot parse resource location: " + m_resources_location_str + "\n", fmi2Status::fmi2Warning,
                  "logStatusWarning");

        m_resources_location = GetLibraryLocation() + "/../../resources/";
        sendToLog("Rolled back to default location: " + m_resources_location + "\n", fmi2Status::fmi2Warning,
                  "logStatusWarning");
    }

    //// Compare GUID
    if (std::string(fmuGUID).compare(m_fmuGUID)) {
        sendToLog("GUID used for instantiation not matching with source.\n", fmi2Status::fmi2Warning,
                  "logStatusWarning");
    }

    for (auto deb_sel = logCategories_debug_init.begin(); deb_sel != logCategories_debug_init.end(); ++deb_sel) {
        if (logCategories_init.find(*deb_sel) == logCategories_init.end()) {
            sendToLog("Developer error: Log category \"" + *deb_sel +
                          "\" specified to be of debug is not listed as a log category.\n",
                      fmi2Status::fmi2Warning, "logStatusWarning");
        }
    }
}

void FmuComponentBase::initializeType(fmi2Type fmuType) {
    switch (fmuType) {
        case fmi2Type::fmi2CoSimulation:
            if (!is_cosimulation_available())
                throw std::runtime_error("Requested CoSimulation FMU mode but it is not available.");
            this->m_fmuType = fmi2Type::fmi2CoSimulation;
            break;
        case fmi2Type::fmi2ModelExchange:
            if (!is_modelexchange_available())
                throw std::runtime_error("Requested ModelExchange FMU mode but it is not available.");
            this->m_fmuType = fmi2Type::fmi2ModelExchange;
            break;
        default:
            throw std::runtime_error("Requested unrecognized FMU type.");
            break;
    }
}

void FmuComponentBase::SetDefaultExperiment(fmi2Boolean toleranceDefined,
                                            fmi2Real tolerance,
                                            fmi2Real startTime,
                                            fmi2Boolean stopTimeDefined,
                                            fmi2Real stopTime) {
    m_startTime = startTime;
    m_stopTime = stopTime;
    m_tolerance = tolerance;
    m_toleranceDefined = toleranceDefined;
    m_stopTimeDefined = stopTimeDefined;
}

void FmuComponentBase::EnterInitializationMode() {
    m_fmuMachineState = FmuMachineStateType::initializationMode;
    _enterInitializationMode();
}

void FmuComponentBase::ExitInitializationMode() {
    _exitInitializationMode();
    m_fmuMachineState = FmuMachineStateType::stepCompleted;  // TODO: introduce additional state when after
                                                             // initialization and before step?
}

void FmuComponentBase::SetDebugLogging(std::string cat, bool value) {
    try {
        m_logCategories_enabled[cat] = value;
    } catch (const std::exception& ex) {
        sendToLog("The LogCategory \"" + cat +
                      "\" is not recognized by the FMU. Please check its availability in modelDescription.xml.\n",
                  fmi2Error, "logStatusError");
    }
}

//// Developer Note: unfortunately it is not possible to retrieve the fmi2 type based on the var_ptr only; the
/// reason is
/// as / follows: e.g. both fmi2Integer and fmi2Boolean are actually alias of type int, thus impeding any
/// possible / splitting depending on type if we accept to have both fmi2Integer and fmi2Boolean considered as
/// the same type we / can drop the 'scalartype' argument but the risk is that a variable might end up being
/// flagged as Integer while / it's actually a Boolean and it is not nice. At least, in this way, we do not have
/// any redundant code.
const FmuVariableExport& FmuComponentBase::AddFmuVariable(const FmuVariableExport::VarbindType& varbind,
                                                          std::string name,
                                                          FmuVariable::Type scalartype,
                                                          std::string unitname,
                                                          std::string description,
                                                          FmuVariable::CausalityType causality,
                                                          FmuVariable::VariabilityType variability,
                                                          FmuVariable::InitialType initial) {
    // check if unit definition exists
    auto match_unit = m_unitDefinitions.find(unitname);
    if (match_unit == m_unitDefinitions.end()) {
        auto predicate_samename = [unitname](const UnitDefinitionType& var) { return var.name == unitname; };
        auto match_commonunit =
            std::find_if(common_unitdefinitions.begin(), common_unitdefinitions.end(), predicate_samename);
        if (match_commonunit == common_unitdefinitions.end()) {
            throw std::runtime_error(
                "Variable unit is not registered within this FmuComponentBase. Call 'addUnitDefinition' "
                "first.");
        } else {
            addUnitDefinition(*match_commonunit);
        }
    }

    // create new variable
    // check if same-name variable exists
    std::set<FmuVariableExport>::iterator it = this->findByName(name);
    if (it != m_scalarVariables.end())
        throw std::runtime_error("Cannot add two FMU variables with the same name.");

    FmuVariableExport newvar(varbind, name, scalartype, causality, variability, initial);
    newvar.SetUnitName(unitname);
    newvar.SetValueReference(++m_valueReferenceCounter[scalartype]);
    newvar.SetDescription(description);

    // check that the attributes of the variable would allow a no-set variable
    const FmuMachineStateType tempFmuState = FmuMachineStateType::anySettableState;

    newvar.ExposeCurrentValueAsStart();
    // varns::visit([&newvar](auto var_ptr_expanded) { newvar.SetStartValIfRequired(var_ptr_expanded);},
    // var_ptr);

    std::pair<std::set<FmuVariableExport>::iterator, bool> ret = m_scalarVariables.insert(newvar);
    if (!ret.second)
        throw std::runtime_error("Developer error: cannot insert new variable into FMU.");

    return *(ret.first);
}

bool FmuComponentBase::RebindVariable(FmuVariableExport::VarbindType varbind, std::string name) {
    std::set<FmuVariableExport>::iterator it = this->findByName(name);
    if (it != m_scalarVariables.end()) {
        FmuVariableExport newvar(*it);
        newvar.Bind(varbind);
        m_scalarVariables.erase(*it);

        std::pair<std::set<FmuVariableExport>::iterator, bool> ret = m_scalarVariables.insert(newvar);

        return ret.second;
    }

    return false;
}

void FmuComponentBase::ExportModelDescription(std::string path) {
    _preModelDescriptionExport();

    // Create the XML document
    rapidxml::xml_document<>* doc_ptr = new rapidxml::xml_document<>();

    // Add the XML declaration
    rapidxml::xml_node<>* declaration = doc_ptr->allocate_node(rapidxml::node_declaration);
    declaration->append_attribute(doc_ptr->allocate_attribute("version", "1.0"));
    declaration->append_attribute(doc_ptr->allocate_attribute("encoding", "UTF-8"));
    doc_ptr->append_node(declaration);

    // Create the root node
    rapidxml::xml_node<>* rootNode = doc_ptr->allocate_node(rapidxml::node_element, "fmiModelDescription");
    rootNode->append_attribute(doc_ptr->allocate_attribute("xmlns:xsi", "http://www.w3.org/2001/XMLSchema-instance"));
    rootNode->append_attribute(doc_ptr->allocate_attribute("fmiVersion", fmi2GetVersion()));
    rootNode->append_attribute(
        doc_ptr->allocate_attribute("modelName", m_modelIdentifier.c_str()));  // modelName can be anything else
    rootNode->append_attribute(doc_ptr->allocate_attribute("guid", m_fmuGUID.c_str()));
    rootNode->append_attribute(doc_ptr->allocate_attribute("generationTool", "rapidxml"));
    rootNode->append_attribute(doc_ptr->allocate_attribute("variableNamingConvention", "structured"));
    rootNode->append_attribute(doc_ptr->allocate_attribute("numberOfEventIndicators", "0"));
    doc_ptr->append_node(rootNode);

    // Add CoSimulation node
    if (is_cosimulation_available()) {
        rapidxml::xml_node<>* coSimNode = doc_ptr->allocate_node(rapidxml::node_element, "CoSimulation");
        coSimNode->append_attribute(doc_ptr->allocate_attribute("modelIdentifier", m_modelIdentifier.c_str()));
        coSimNode->append_attribute(doc_ptr->allocate_attribute("canHandleVariableCommunicationStepSize", "true"));
        coSimNode->append_attribute(doc_ptr->allocate_attribute("canInterpolateInputs", "true"));
        coSimNode->append_attribute(doc_ptr->allocate_attribute("maxOutputDerivativeOrder", "1"));
        coSimNode->append_attribute(doc_ptr->allocate_attribute("canGetAndSetFMUstate", "false"));
        coSimNode->append_attribute(doc_ptr->allocate_attribute("canSerializeFMUstate", "false"));
        coSimNode->append_attribute(doc_ptr->allocate_attribute("providesDirectionalDerivative", "false"));
        rootNode->append_node(coSimNode);
    }

    // Add ModelExchange node
    if (is_modelexchange_available()) {
        rapidxml::xml_node<>* modExNode = doc_ptr->allocate_node(rapidxml::node_element, "ModelExchange");
        rootNode->append_node(modExNode);
        throw std::runtime_error("Developer error: ModelExchange not supported in modelDescription.xml");
    }

    // Add UnitDefinitions node
    std::list<std::string> stringbuf;
    rapidxml::xml_node<>* unitDefsNode = doc_ptr->allocate_node(rapidxml::node_element, "UnitDefinitions");

    for (auto& ud_pair : m_unitDefinitions) {
        auto& ud = ud_pair.second;
        rapidxml::xml_node<>* unitNode = doc_ptr->allocate_node(rapidxml::node_element, "Unit");
        unitNode->append_attribute(doc_ptr->allocate_attribute("name", ud.name.c_str()));

        rapidxml::xml_node<>* baseUnitNode = doc_ptr->allocate_node(rapidxml::node_element, "BaseUnit");
        if (ud.kg != 0) {
            stringbuf.push_back(std::to_string(ud.kg));
            baseUnitNode->append_attribute(doc_ptr->allocate_attribute("kg", stringbuf.back().c_str()));
        }
        if (ud.m != 0) {
            stringbuf.push_back(std::to_string(ud.m));
            baseUnitNode->append_attribute(doc_ptr->allocate_attribute("m", stringbuf.back().c_str()));
        }
        if (ud.s != 0) {
            stringbuf.push_back(std::to_string(ud.s));
            baseUnitNode->append_attribute(doc_ptr->allocate_attribute("s", stringbuf.back().c_str()));
        }
        if (ud.A != 0) {
            stringbuf.push_back(std::to_string(ud.A));
            baseUnitNode->append_attribute(doc_ptr->allocate_attribute("A", stringbuf.back().c_str()));
        }
        if (ud.K != 0) {
            stringbuf.push_back(std::to_string(ud.K));
            baseUnitNode->append_attribute(doc_ptr->allocate_attribute("K", stringbuf.back().c_str()));
        }
        if (ud.mol != 0) {
            stringbuf.push_back(std::to_string(ud.mol));
            baseUnitNode->append_attribute(doc_ptr->allocate_attribute("mol", stringbuf.back().c_str()));
        }
        if (ud.cd != 0) {
            stringbuf.push_back(std::to_string(ud.cd));
            baseUnitNode->append_attribute(doc_ptr->allocate_attribute("cd", stringbuf.back().c_str()));
        }
        if (ud.rad != 0) {
            stringbuf.push_back(std::to_string(ud.rad));
            baseUnitNode->append_attribute(doc_ptr->allocate_attribute("rad", stringbuf.back().c_str()));
        }
        unitNode->append_node(baseUnitNode);

        unitDefsNode->append_node(unitNode);
    }
    rootNode->append_node(unitDefsNode);

    // Add LogCategories node
    rapidxml::xml_node<>* logCategoriesNode = doc_ptr->allocate_node(rapidxml::node_element, "LogCategories");
    for (auto& lc : m_logCategories_enabled) {
        rapidxml::xml_node<>* logCategoryNode = doc_ptr->allocate_node(rapidxml::node_element, "Category");
        logCategoryNode->append_attribute(doc_ptr->allocate_attribute("name", lc.first.c_str()));
        logCategoryNode->append_attribute(doc_ptr->allocate_attribute(
            "description", m_logCategories_debug.find(lc.first) == m_logCategories_debug.end() ? "NotDebugCategory"
                                                                                               : "DebugCategory"));
        logCategoriesNode->append_node(logCategoryNode);
    }
    rootNode->append_node(logCategoriesNode);

    // Add DefaultExperiment node
    std::string startTime_str = std::to_string(m_startTime);
    std::string stopTime_str = std::to_string(m_stopTime);
    std::string stepSize_str = std::to_string(m_stepSize);
    std::string tolerance_str = std::to_string(m_tolerance);

    rapidxml::xml_node<>* defaultExpNode = doc_ptr->allocate_node(rapidxml::node_element, "DefaultExperiment");
    defaultExpNode->append_attribute(doc_ptr->allocate_attribute("startTime", startTime_str.c_str()));
    defaultExpNode->append_attribute(doc_ptr->allocate_attribute("stopTime", stopTime_str.c_str()));
    if (m_stepSize > 0)
        defaultExpNode->append_attribute(doc_ptr->allocate_attribute("stepSize", stepSize_str.c_str()));
    if (m_tolerance > 0)
        defaultExpNode->append_attribute(doc_ptr->allocate_attribute("tolerance", tolerance_str.c_str()));
    rootNode->append_node(defaultExpNode);

    // Add ModelVariables node
    rapidxml::xml_node<>* modelVarsNode = doc_ptr->allocate_node(rapidxml::node_element, "ModelVariables");

    // WARNING: rapidxml does not copy the strings that we pass to print, but it just keeps the addresses until
    // it's time to print them so we cannot use a temporary string to convert the number to string and then
    // recycle it we cannot use std::vector because, in case of reallocation, it might move the array somewhere
    // else thus invalidating the addresses
    std::list<std::string> valueref_str;

    // TODO: move elsewhere
    const std::unordered_map<FmuVariable::Type, std::string> Type_strings = {{FmuVariable::Type::Real, "Real"},
                                                                             {FmuVariable::Type::Integer, "Integer"},
                                                                             {FmuVariable::Type::Boolean, "Boolean"},
                                                                             {FmuVariable::Type::Unknown, "Unknown"},
                                                                             {FmuVariable::Type::String, "String"}};

    const std::unordered_map<FmuVariable::InitialType, std::string> InitialType_strings = {
        {FmuVariable::InitialType::exact, "exact"},
        {FmuVariable::InitialType::approx, "approx"},
        {FmuVariable::InitialType::calculated, "calculated"}};

    const std::unordered_map<FmuVariable::VariabilityType, std::string> VariabilityType_strings = {
        {FmuVariable::VariabilityType::constant, "constant"},
        {FmuVariable::VariabilityType::fixed, "fixed"},
        {FmuVariable::VariabilityType::tunable, "tunable"},
        {FmuVariable::VariabilityType::discrete, "discrete"},
        {FmuVariable::VariabilityType::continuous, "continuous"}};

    const std::unordered_map<FmuVariable::CausalityType, std::string> CausalityType_strings = {
        {FmuVariable::CausalityType::parameter, "parameter"},
        {FmuVariable::CausalityType::calculatedParameter, "calculatedParameter"},
        {FmuVariable::CausalityType::input, "input"},
        {FmuVariable::CausalityType::output, "output"},
        {FmuVariable::CausalityType::local, "local"},
        {FmuVariable::CausalityType::independent, "independent"}};

    for (std::set<FmuVariableExport>::const_iterator it = m_scalarVariables.begin(); it != m_scalarVariables.end();
         ++it) {
        // Create a ScalarVariable node
        rapidxml::xml_node<>* scalarVarNode = doc_ptr->allocate_node(rapidxml::node_element, "ScalarVariable");
        scalarVarNode->append_attribute(doc_ptr->allocate_attribute("name", it->GetName().c_str()));

        valueref_str.push_back(std::to_string(it->GetValueReference()));
        scalarVarNode->append_attribute(doc_ptr->allocate_attribute("valueReference", valueref_str.back().c_str()));

        if (!it->GetDescription().empty())
            scalarVarNode->append_attribute(doc_ptr->allocate_attribute("description", it->GetDescription().c_str()));
        if (it->GetCausality() != FmuVariable::CausalityType::local)
            scalarVarNode->append_attribute(
                doc_ptr->allocate_attribute("causality", CausalityType_strings.at(it->GetCausality()).c_str()));
        if (it->GetVariability() != FmuVariable::VariabilityType::continuous)
            scalarVarNode->append_attribute(
                doc_ptr->allocate_attribute("variability", VariabilityType_strings.at(it->GetVariability()).c_str()));
        if (it->GetInitial() != FmuVariable::InitialType::none)
            scalarVarNode->append_attribute(
                doc_ptr->allocate_attribute("initial", InitialType_strings.at(it->GetInitial()).c_str()));
        modelVarsNode->append_node(scalarVarNode);

        rapidxml::xml_node<>* unitNode =
            doc_ptr->allocate_node(rapidxml::node_element, Type_strings.at(it->GetType()).c_str());
        if (it->GetType() == FmuVariable::Type::Real && !it->GetUnitName().empty())
            unitNode->append_attribute(doc_ptr->allocate_attribute("unit", it->GetUnitName().c_str()));
        if (it->HasStartVal()) {
            stringbuf.push_back(it->GetStartVal_toString());
            unitNode->append_attribute(doc_ptr->allocate_attribute("start", stringbuf.back().c_str()));
        }
        scalarVarNode->append_node(unitNode);
    }

    rootNode->append_node(modelVarsNode);

    // Add ModelStructure node
    rapidxml::xml_node<>* modelStructNode = doc_ptr->allocate_node(rapidxml::node_element, "ModelStructure");
    // TODO: Add Outputs, Derivatives, and InitialUnknowns nodes and attributes here...
    rootNode->append_node(modelStructNode);

    // Save the XML document to a file
    std::ofstream outFile(path + "/modelDescription.xml");
    outFile << *doc_ptr;
    outFile.close();

    delete doc_ptr;

    _postModelDescriptionExport();
}

std::set<FmuVariableExport>::iterator FmuComponentBase::findByValrefType(fmi2ValueReference vr,
                                                                         FmuVariable::Type vartype) {
    auto predicate_samevalreftype = [vr, vartype](const FmuVariable& var) {
        return var.GetValueReference() == vr && var.GetType() == vartype;
    };
    return std::find_if(m_scalarVariables.begin(), m_scalarVariables.end(), predicate_samevalreftype);
}

void FmuComponentBase::executePreStepCallbacks() {
    for (auto& callb : m_preStepCallbacks) {
        callb();
    }
}

void FmuComponentBase::executePostStepCallbacks() {
    for (auto& callb : m_postStepCallbacks) {
        callb();
    }
}

std::set<FmuVariableExport>::iterator FmuComponentBase::findByName(const std::string& name) {
    auto predicate_samename = [name](const FmuVariable& var) { return !var.GetName().compare(name); };
    return std::find_if(m_scalarVariables.begin(), m_scalarVariables.end(), predicate_samename);
}

fmi2Status FmuComponentBase::DoStep(fmi2Real currentCommunicationPoint,
                                    fmi2Real communicationStepSize,
                                    fmi2Boolean noSetFMUStatePriorToCurrentPoint) {
    // invoke any pre step callbacks (e.g., to process input variables)
    executePreStepCallbacks();

    fmi2Status doStep_status =
        _doStep(currentCommunicationPoint, communicationStepSize, noSetFMUStatePriorToCurrentPoint);

    // invoke any post step callbacks (e.g., to update auxiliary variables)
    executePostStepCallbacks();

    switch (doStep_status) {
        case fmi2OK:
            m_fmuMachineState = FmuMachineStateType::stepCompleted;
            break;
        case fmi2Warning:
            m_fmuMachineState = FmuMachineStateType::stepCompleted;
            break;
        case fmi2Discard:
            m_fmuMachineState = FmuMachineStateType::stepFailed;
            break;
        case fmi2Error:
            m_fmuMachineState = FmuMachineStateType::error;
            break;
        case fmi2Fatal:
            m_fmuMachineState = FmuMachineStateType::fatal;
            break;
        case fmi2Pending:
            m_fmuMachineState = FmuMachineStateType::stepInProgress;
            break;
        default:
            throw std::runtime_error("Developer error: unexpected status from _doStep");
            break;
    }

    return doStep_status;
}

void FmuComponentBase::addUnitDefinition(const UnitDefinitionType& unit_definition) {
    m_unitDefinitions[unit_definition.name] = unit_definition;
}

void FmuComponentBase::sendToLog(std::string msg, fmi2Status status, std::string msg_cat) {
    assert(m_logCategories_enabled.find(msg_cat) != m_logCategories_enabled.end() &&
           ("Developer warning: the category \"" + msg_cat + "\" is not recognized by the FMU").c_str());

    if (m_logCategories_enabled.find(msg_cat) == m_logCategories_enabled.end() || m_logCategories_enabled[msg_cat] ||
        (m_debug_logging_enabled && m_logCategories_debug.find(msg_cat) != m_logCategories_debug.end())) {
        m_callbackFunctions.logger(m_callbackFunctions.componentEnvironment, m_instanceName.c_str(), status,
                                   msg_cat.c_str(), msg.c_str());
    }
}

// =============================================================================
// FMU FUNCTIONS
// =============================================================================

fmi2Component fmi2Instantiate(fmi2String instanceName,
                              fmi2Type fmuType,
                              fmi2String fmuGUID,
                              fmi2String fmuResourceLocation,
                              const fmi2CallbackFunctions* functions,
                              fmi2Boolean visible,
                              fmi2Boolean loggingOn) {
    FmuComponentBase* fmu_ptr =
        fmi2Instantiate_getPointer(instanceName, fmuType, fmuGUID, fmuResourceLocation, functions, visible, loggingOn);
    return reinterpret_cast<void*>(fmu_ptr);
}

const char* fmi2GetTypesPlatform(void) {
    return fmi2TypesPlatform;
}

const char* fmi2GetVersion(void) {
    return fmi2Version;
}

fmi2Status fmi2SetDebugLogging(fmi2Component c,
                               fmi2Boolean loggingOn,
                               size_t nCategories,
                               const fmi2String categories[]) {
    FmuComponentBase* fmu_ptr = reinterpret_cast<FmuComponentBase*>(c);
    for (auto cs = 0; cs < nCategories; ++cs) {
        fmu_ptr->SetDebugLogging(categories[cs], loggingOn == fmi2True ? true : false);
    }

    return fmi2Status::fmi2OK;
}

void fmi2FreeInstance(fmi2Component c) {
    delete reinterpret_cast<FmuComponentBase*>(c);
}

fmi2Status fmi2SetupExperiment(fmi2Component c,
                               fmi2Boolean toleranceDefined,
                               fmi2Real tolerance,
                               fmi2Real startTime,
                               fmi2Boolean stopTimeDefined,
                               fmi2Real stopTime) {
    reinterpret_cast<FmuComponentBase*>(c)->SetDefaultExperiment(toleranceDefined, tolerance, startTime,
                                                                 stopTimeDefined, stopTime);
    return fmi2Status::fmi2OK;
}

fmi2Status fmi2EnterInitializationMode(fmi2Component c) {
    reinterpret_cast<FmuComponentBase*>(c)->EnterInitializationMode();
    return fmi2Status::fmi2OK;
}

fmi2Status fmi2ExitInitializationMode(fmi2Component c) {
    reinterpret_cast<FmuComponentBase*>(c)->ExitInitializationMode();
    return fmi2Status::fmi2OK;
}
fmi2Status fmi2Terminate(fmi2Component c) {
    return fmi2Status::fmi2OK;
}
fmi2Status fmi2Reset(fmi2Component c) {
    return fmi2Status::fmi2OK;
}

fmi2Status fmi2GetReal(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2Real value[]) {
    return reinterpret_cast<FmuComponentBase*>(c)->fmi2GetVariable(vr, nvr, value, FmuVariable::Type::Real);
}

fmi2Status fmi2GetInteger(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2Integer value[]) {
    return reinterpret_cast<FmuComponentBase*>(c)->fmi2GetVariable(vr, nvr, value, FmuVariable::Type::Integer);
}

fmi2Status fmi2GetBoolean(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2Boolean value[]) {
    return reinterpret_cast<FmuComponentBase*>(c)->fmi2GetVariable(vr, nvr, value, FmuVariable::Type::Boolean);
}

fmi2Status fmi2GetString(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2String value[]) {
    return reinterpret_cast<FmuComponentBase*>(c)->fmi2GetVariable(vr, nvr, value, FmuVariable::Type::String);
}

fmi2Status fmi2SetReal(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2Real value[]) {
    return reinterpret_cast<FmuComponentBase*>(c)->fmi2SetVariable(vr, nvr, value, FmuVariable::Type::Real);
}

fmi2Status fmi2SetInteger(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2Integer value[]) {
    return reinterpret_cast<FmuComponentBase*>(c)->fmi2SetVariable(vr, nvr, value, FmuVariable::Type::Integer);
}

fmi2Status fmi2SetBoolean(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2Boolean value[]) {
    return reinterpret_cast<FmuComponentBase*>(c)->fmi2SetVariable(vr, nvr, value, FmuVariable::Type::Boolean);
}

fmi2Status fmi2SetString(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2String value[]) {
    return reinterpret_cast<FmuComponentBase*>(c)->fmi2SetVariable(vr, nvr, value, FmuVariable::Type::String);
}

fmi2Status fmi2GetFMUstate(fmi2Component c, fmi2FMUstate* FMUstate) {
    return fmi2Status::fmi2OK;
}
fmi2Status fmi2SetFMUstate(fmi2Component c, fmi2FMUstate FMUstate) {
    return fmi2Status::fmi2OK;
}
fmi2Status fmi2FreeFMUstate(fmi2Component c, fmi2FMUstate* FMUstate) {
    return fmi2Status::fmi2OK;
}
fmi2Status fmi2SerializedFMUstateSize(fmi2Component c, fmi2FMUstate FMUstate, size_t* size) {
    return fmi2Status::fmi2OK;
}
fmi2Status fmi2SerializeFMUstate(fmi2Component c, fmi2FMUstate FMUstate, fmi2Byte serializedState[], size_t size) {
    return fmi2Status::fmi2OK;
}
fmi2Status fmi2DeSerializeFMUstate(fmi2Component c,
                                   const fmi2Byte serializedState[],
                                   size_t size,
                                   fmi2FMUstate* FMUstate) {
    return fmi2Status::fmi2OK;
}
fmi2Status fmi2GetDirectionalDerivative(fmi2Component c,
                                        const fmi2ValueReference vUnknown_ref[],
                                        size_t nUnknown,
                                        const fmi2ValueReference vKnown_ref[],
                                        size_t nKnown,
                                        const fmi2Real dvKnown[],
                                        fmi2Real dvUnknown[]) {
    return fmi2Status::fmi2OK;
}

////// Model Exchange
// fmi2Status fmi2EnterEventMode(fmi2Component c){ return fmi2Status::fmi2OK; }
// fmi2Status fmi2NewDiscreteStates(fmi2Component c, fmi2EventInfo* fmi2eventInfo){ return fmi2Status::fmi2OK; }
// fmi2Status fmi2EnterContinuousTimeMode(fmi2Component c){ return fmi2Status::fmi2OK; }
// fmi2Status fmi2CompletedIntegratorStep(fmi2Component c, fmi2Boolean noSetFMUStatePriorToCurrentPoint,
// fmi2Boolean* enterEventMode, fmi2Boolean* terminateSimulation){ return fmi2Status::fmi2OK; } fmi2Status
// fmi2SetTime(fmi2Component c, fmi2Real time){ return fmi2Status::fmi2OK; } fmi2Status
// fmi2SetContinuousStates(fmi2Component c, const fmi2Real x[], size_t nx){ return fmi2Status::fmi2OK; }
// fmi2Status fmi2GetDerivatives(fmi2Component c, fmi2Real derivatives[], size_t nx){ return fmi2Status::fmi2OK;
// } fmi2Status fmi2GetEventIndicators(fmi2Component c, fmi2Real eventIndicators[], size_t ni){ return
// fmi2Status::fmi2OK; } fmi2Status fmi2GetContinuousStates(fmi2Component c, fmi2Real x[], size_t nx){ return
// fmi2Status::fmi2OK; } fmi2Status fmi2GetNominalsOfContinuousStates(fmi2Component c, fmi2Real x_nominal[],
// size_t nx){ return fmi2Status::fmi2OK; }

// Co-Simulation
fmi2Status fmi2SetRealInputDerivatives(fmi2Component c,
                                       const fmi2ValueReference vr[],
                                       size_t nvr,
                                       const fmi2Integer order[],
                                       const fmi2Real value[]) {
    return fmi2Status::fmi2OK;
}
fmi2Status fmi2GetRealOutputDerivatives(fmi2Component c,
                                        const fmi2ValueReference vr[],
                                        size_t nvr,
                                        const fmi2Integer order[],
                                        fmi2Real value[]) {
    return fmi2Status::fmi2OK;
}

fmi2Status fmi2DoStep(fmi2Component c,
                      fmi2Real currentCommunicationPoint,
                      fmi2Real communicationStepSize,
                      fmi2Boolean noSetFMUStatePriorToCurrentPoint) {
    return reinterpret_cast<FmuComponentBase*>(c)->DoStep(currentCommunicationPoint, communicationStepSize,
                                                          noSetFMUStatePriorToCurrentPoint);
}

fmi2Status fmi2CancelStep(fmi2Component c) {
    return fmi2Status::fmi2OK;
}
fmi2Status fmi2GetStatus(fmi2Component c, const fmi2StatusKind s, fmi2Status* value) {
    return fmi2Status::fmi2OK;
}
fmi2Status fmi2GetRealStatus(fmi2Component c, const fmi2StatusKind s, fmi2Real* value) {
    return fmi2Status::fmi2OK;
}
fmi2Status fmi2GetIntegerStatus(fmi2Component c, const fmi2StatusKind s, fmi2Integer* value) {
    return fmi2Status::fmi2OK;
}
fmi2Status fmi2GetBooleanStatus(fmi2Component c, const fmi2StatusKind s, fmi2Boolean* value) {
    return fmi2Status::fmi2OK;
}
fmi2Status fmi2GetStringStatus(fmi2Component c, const fmi2StatusKind s, fmi2String* value) {
    return fmi2Status::fmi2OK;
}
