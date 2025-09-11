/**
 * @file capability_manager.cpp
 * @brief 机器人能力管理器的实现
 * @author Claude Code
 * @date 2024
 */

#include "robot_factory/capability_manager/capability_manager.hpp"
#include <filesystem>
#include <algorithm>
#include <set>

namespace robot_factory {
namespace capability_manager {

CapabilityManager::CapabilityManager() {
    // Basic initialization
}

bool CapabilityManager::initialize(const std::string& config_directory) {
    config_directory_ = config_directory;
    loadDefaultProfiles();
    initialized_ = true;
    return true;
}

bool CapabilityManager::shutdown() {
    initialized_ = false;
    capability_profiles_.clear();
    capability_requirements_.clear();
    return true;
}

bool CapabilityManager::loadCapabilityProfile(const std::string& file_path) {
    // TODO: Implement file loading
    setLastError("File loading not yet implemented");
    return false;
}

bool CapabilityManager::registerCapabilityProfile(const RobotCapabilityProfile& profile) {
    if (!validateCapabilityProfile(profile)) {
        setLastError("Invalid capability profile");
        return false;
    }
    
    capability_profiles_[profile.robot_type] = std::make_shared<RobotCapabilityProfile>(profile);
    return true;
}

std::shared_ptr<RobotCapabilityProfile> CapabilityManager::getCapabilityProfile(RobotType robot_type) const {
    auto it = capability_profiles_.find(robot_type);
    if (it != capability_profiles_.end()) {
        return it->second;
    }
    return nullptr;
}

std::vector<RobotType> CapabilityManager::getRegisteredRobotTypes() const {
    std::vector<RobotType> types;
    for (const auto& pair : capability_profiles_) {
        types.push_back(pair.first);
    }
    return types;
}

bool CapabilityManager::updateCapabilityProfile(RobotType robot_type, const RobotCapabilityProfile& profile) {
    if (capability_profiles_.find(robot_type) == capability_profiles_.end()) {
        setLastError("Robot type not found");
        return false;
    }
    
    if (!validateCapabilityProfile(profile)) {
        setLastError("Invalid capability profile");
        return false;
    }
    
    capability_profiles_[robot_type] = std::make_shared<RobotCapabilityProfile>(profile);
    return true;
}

bool CapabilityManager::hasCapability(RobotType robot_type, const std::string& capability_id) const {
    auto profile = getCapabilityProfile(robot_type);
    if (!profile) {
        return false;
    }
    
    for (const auto& capability : profile->capabilities) {
        if (capability.capability_id == capability_id) {
            return true;
        }
    }
    return false;
}

std::shared_ptr<CapabilityDefinition> CapabilityManager::getCapabilityDefinition(
    RobotType robot_type, const std::string& capability_id) const {
    
    auto profile = getCapabilityProfile(robot_type);
    if (!profile) {
        return nullptr;
    }
    
    for (const auto& capability : profile->capabilities) {
        if (capability.capability_id == capability_id) {
            return std::make_shared<CapabilityDefinition>(capability);
        }
    }
    return nullptr;
}

std::vector<CapabilityDefinition> CapabilityManager::getRobotCapabilities(
    RobotType robot_type, CapabilityType capability_type) const {
    
    std::vector<CapabilityDefinition> result;
    auto profile = getCapabilityProfile(robot_type);
    if (!profile) {
        return result;
    }
    
    for (const auto& capability : profile->capabilities) {
        if (capability_type == CapabilityType::CUSTOM || capability.type == capability_type) {
            result.push_back(capability);
        }
    }
    return result;
}

std::string CapabilityManager::compareRobotCapabilities(RobotType robot_type1, RobotType robot_type2) const {
    // TODO: Implement comparison logic
    return "Comparison not yet implemented";
}

CapabilityMatchResult CapabilityManager::matchCapabilityRequirement(
    RobotType robot_type, const CapabilityRequirement& requirement) const {
    
    CapabilityMatchResult result;
    auto profile = getCapabilityProfile(robot_type);
    if (!profile) {
        result.is_compatible = false;
        result.compatibility_score = 0.0f;
        result.incompatibility_reasons.push_back("Robot type not found");
        return result;
    }
    
    // 执行详细的能力匹配分析
    performDetailedCapabilityAnalysis(*profile, requirement, result);
    
    // 计算综合兼容性评分
    result.compatibility_score = calculateCompatibilityScore(*profile, requirement);
    
    // 判断是否兼容
    result.is_compatible = result.compatibility_score >= requirement.minimum_score;
    
    // 生成推荐建议
    if (!result.is_compatible) {
        generateImprovementRecommendations(*profile, requirement, result);
    }
    
    return result;
}

std::vector<std::pair<RobotType, CapabilityMatchResult>> CapabilityManager::findBestMatchingRobots(
    const CapabilityRequirement& requirement) const {
    
    std::vector<std::pair<RobotType, CapabilityMatchResult>> results;
    
    for (const auto& pair : capability_profiles_) {
        auto match_result = matchCapabilityRequirement(pair.first, requirement);
        results.emplace_back(pair.first, match_result);
    }
    
    // Sort by compatibility score
    std::sort(results.begin(), results.end(),
        [](const auto& a, const auto& b) {
            return a.second.compatibility_score > b.second.compatibility_score;
        });
    
    return results;
}

CapabilityMatchResult CapabilityManager::matchCollaborativeCapability(
    const std::vector<RobotType>& robot_types, 
    const CapabilityRequirement& requirement) const {
    
    CapabilityMatchResult result;
    result.is_compatible = true;
    result.compatibility_score = 0.0f;
    
    if (robot_types.empty()) {
        result.is_compatible = false;
        result.incompatibility_reasons.push_back("No robots specified for collaborative matching");
        return result;
    }
    
    // 收集所有机器人的能力
    std::vector<RobotCapabilityProfile> profiles;
    std::vector<float> individual_scores;
    
    for (RobotType robot_type : robot_types) {
        auto profile = getCapabilityProfile(robot_type);
        if (!profile) {
            result.is_compatible = false;
            result.incompatibility_reasons.push_back(
                "Robot type " + std::to_string(static_cast<int>(robot_type)) + " not found");
            continue;
        }
        
        profiles.push_back(*profile);
        
        // 计算每个机器人的个人兼容性评分
        float individual_score = calculateCompatibilityScore(*profile, requirement);
        individual_scores.push_back(individual_score);
    }
    
    if (profiles.empty()) {
        result.compatibility_score = 0.0f;
        return result;
    }
    
    // 协作评分策略
    float collaborative_score = calculateCollaborativeScore(profiles, requirement, individual_scores);
    result.compatibility_score = collaborative_score;
    result.is_compatible = collaborative_score >= requirement.minimum_score;
    
    // 分析协作能力互补性
    analyzeCapabilityComplementarity(profiles, requirement, result);
    
    return result;
}

bool CapabilityManager::registerCapabilityRequirement(const CapabilityRequirement& requirement) {
    capability_requirements_[requirement.requirement_id] = std::make_shared<CapabilityRequirement>(requirement);
    return true;
}

std::shared_ptr<CapabilityRequirement> CapabilityManager::getCapabilityRequirement(
    const std::string& requirement_id) const {
    
    auto it = capability_requirements_.find(requirement_id);
    if (it != capability_requirements_.end()) {
        return it->second;
    }
    return nullptr;
}

std::vector<CapabilityRequirement> CapabilityManager::getAllCapabilityRequirements() const {
    std::vector<CapabilityRequirement> requirements;
    for (const auto& pair : capability_requirements_) {
        requirements.push_back(*pair.second);
    }
    return requirements;
}

RobotCapabilityProfile CapabilityManager::createGo2DefaultProfile() {
    RobotCapabilityProfile profile;
    profile.robot_type = static_cast<RobotType>(1); // Assuming Go2 = 1
    profile.robot_name = "Unitree Go2";
    profile.robot_model = "Go2";
    profile.profile_version = "1.0.0";
    
    // Basic physical properties for Go2
    profile.physical_properties.length = 0.845f;
    profile.physical_properties.width = 0.405f;
    profile.physical_properties.height = 0.320f;
    profile.physical_properties.weight = 15.0f;
    profile.physical_properties.max_payload = 5.0f;
    
    // TODO: Add more Go2-specific capabilities
    
    return profile;
}

CapabilityRequirement CapabilityManager::createNavigationRequirement() {
    CapabilityRequirement requirement("navigation", "Basic Navigation");
    requirement.description = "Basic navigation capabilities including SLAM and path planning";
    requirement.required_capabilities = {"motion_control", "localization", "path_planning"};
    requirement.optional_capabilities = {"obstacle_avoidance", "mapping"};
    requirement.minimum_score = 0.7f;
    
    return requirement;
}

CapabilityRequirement CapabilityManager::createAutoChargingRequirement() {
    CapabilityRequirement requirement("auto_charging", "Autonomous Charging");
    requirement.description = "Autonomous charging capabilities including docking";
    requirement.required_capabilities = {"power_management", "docking", "charging"};
    requirement.minimum_score = 0.8f;
    
    return requirement;
}

bool CapabilityManager::exportCapabilityProfile(RobotType robot_type, const std::string& file_path,
                                               const std::string& format) const {
    // TODO: Implement export functionality
    setLastError("Export functionality not yet implemented");
    return false;
}

bool CapabilityManager::importCapabilityProfile(const std::string& file_path,
                                               const std::string& format) {
    // TODO: Implement import functionality
    setLastError("Import functionality not yet implemented");
    return false;
}

bool CapabilityManager::generateCapabilityTemplate(RobotType robot_type, const std::string& file_path) const {
    // TODO: Implement template generation
    setLastError("Template generation not yet implemented");
    return false;
}

std::string CapabilityManager::robotTypeToString(RobotType robot_type) {
    // TODO: Implement based on actual RobotType enum values
    return "UNKNOWN_" + std::to_string(static_cast<int>(robot_type));
}

RobotType CapabilityManager::stringToRobotType(const std::string& type_string) {
    // TODO: Implement based on actual RobotType enum values
    return static_cast<RobotType>(0);
}

void CapabilityManager::loadDefaultProfiles() {
    // Create and register default Go2 profile
    auto go2_profile = createGo2DefaultProfile();
    registerCapabilityProfile(go2_profile);
    
    // Register default requirements
    registerCapabilityRequirement(createNavigationRequirement());
    registerCapabilityRequirement(createAutoChargingRequirement());
}

bool CapabilityManager::validateCapabilityProfile(const RobotCapabilityProfile& profile) const {
    if (profile.robot_name.empty() || profile.robot_model.empty()) {
        return false;
    }
    
    if (profile.physical_properties.length <= 0 || 
        profile.physical_properties.width <= 0 || 
        profile.physical_properties.height <= 0 ||
        profile.physical_properties.weight <= 0) {
        return false;
    }
    
    return true;
}

float CapabilityManager::calculateCompatibilityScore(const RobotCapabilityProfile& profile,
                                                   const CapabilityRequirement& requirement) const {
    
    if (requirement.required_capabilities.empty() && requirement.numeric_ranges.empty()) {
        return 1.0f; // 完美评分，如果没有特定要求
    }
    
    float total_score = 0.0f;
    float total_weight = 0.0f;
    
    // 1. 评估必需能力 (权重: 70%)
    float required_score = evaluateRequiredCapabilities(profile, requirement);
    total_score += required_score * 0.7f;
    total_weight += 0.7f;
    
    // 2. 评估数值参数匹配 (权重: 20%)
    float numeric_score = evaluateNumericRequirements(profile, requirement);
    total_score += numeric_score * 0.2f;
    total_weight += 0.2f;
    
    // 3. 评估可选能力 (权重: 10%)
    float optional_score = evaluateOptionalCapabilities(profile, requirement);
    total_score += optional_score * 0.1f;
    total_weight += 0.1f;
    
    // 归一化评分
    return total_weight > 0.0f ? total_score / total_weight : 0.0f;
}

float CapabilityManager::evaluateRequiredCapabilities(
    const RobotCapabilityProfile& profile,
    const CapabilityRequirement& requirement) const {
    
    if (requirement.required_capabilities.empty()) {
        return 1.0f;
    }
    
    int matched_count = 0;
    int total_required = requirement.required_capabilities.size();
    
    for (const auto& req_capability : requirement.required_capabilities) {
        // 检查是否有对应的能力
        if (hasSpecificCapability(profile, req_capability)) {
            float capability_weight = getCapabilityWeight(requirement, req_capability);
            matched_count += capability_weight;
        }
    }
    
    return static_cast<float>(matched_count) / total_required;
}

float CapabilityManager::evaluateNumericRequirements(
    const RobotCapabilityProfile& profile,
    const CapabilityRequirement& requirement) const {
    
    if (requirement.numeric_ranges.empty()) {
        return 1.0f;
    }
    
    float total_score = 0.0f;
    int evaluated_params = 0;
    
    for (const auto& numeric_req : requirement.numeric_ranges) {
        const std::string& param_name = numeric_req.first;
        const auto& range = numeric_req.second;
        
        float robot_value = getRobotNumericParameter(profile, param_name);
        if (robot_value >= 0.0f) { // 有效参数值
            float param_score = calculateNumericScore(robot_value, range.first, range.second);
            total_score += param_score;
            evaluated_params++;
        }
    }
    
    return evaluated_params > 0 ? total_score / evaluated_params : 0.0f;
}

float CapabilityManager::evaluateOptionalCapabilities(
    const RobotCapabilityProfile& profile,
    const CapabilityRequirement& requirement) const {
    
    if (requirement.optional_capabilities.empty()) {
        return 1.0f;
    }
    
    int matched_optional = 0;
    int total_optional = requirement.optional_capabilities.size();
    
    for (const auto& optional_capability : requirement.optional_capabilities) {
        if (hasSpecificCapability(profile, optional_capability)) {
            matched_optional++;
        }
    }
    
    return static_cast<float>(matched_optional) / total_optional;
}

bool CapabilityManager::hasSpecificCapability(
    const RobotCapabilityProfile& profile,
    const std::string& capability_name) const {
    
    // 根据能力名称检查不同类别的能力
    if (capability_name == "motion_control") {
        return profile.capabilities.size() > 0; // 简化检查
    } else if (capability_name == "lidar_sensor") {
        // 这里需要根据实际的 profile 结构进行检查
        // 由于当前结构可能不完整，使用简化逻辑
        return true;
    } else if (capability_name == "navigation") {
        return true; // 假设所有机器人都有基础导航能力
    }
    
    // 通用检查：在能力列表中查找
    for (const auto& capability : profile.capabilities) {
        if (capability.capability_name == capability_name || 
            capability.capability_id == capability_name) {
            return true;
        }
    }
    
    return false;
}

float CapabilityManager::getCapabilityWeight(
    const CapabilityRequirement& requirement,
    const std::string& capability_name) const {
    
    auto weight_it = requirement.capability_weights.find(capability_name);
    if (weight_it != requirement.capability_weights.end()) {
        return weight_it->second;
    }
    
    return 1.0f; // 默认权重
}

float CapabilityManager::getRobotNumericParameter(
    const RobotCapabilityProfile& profile,
    const std::string& parameter_name) const {
    
    // 根据参数名称返回对应的数值
    if (parameter_name == "max_linear_velocity") {
        return profile.physical_properties.max_payload; // 简化示例
    } else if (parameter_name == "max_payload") {
        return profile.physical_properties.max_payload;
    } else if (parameter_name == "weight") {
        return profile.physical_properties.weight;
    }
    
    return -1.0f; // 参数不存在
}

float CapabilityManager::calculateNumericScore(
    float robot_value, float min_required, float max_required) const {
    
    if (robot_value >= min_required && robot_value <= max_required) {
        return 1.0f; // 完美匹配
    } else if (robot_value < min_required) {
        // 低于最小值，评分递减
        float deficit_ratio = (min_required - robot_value) / min_required;
        return std::max(0.0f, 1.0f - deficit_ratio);
    } else {
        // 高于最大值，轻微减分（通常比不足要好）
        float excess_ratio = (robot_value - max_required) / max_required;
        return std::max(0.5f, 1.0f - 0.1f * excess_ratio);
    }
}

void CapabilityManager::performDetailedCapabilityAnalysis(
    const RobotCapabilityProfile& profile,
    const CapabilityRequirement& requirement,
    CapabilityMatchResult& result) const {
    
    // 分析必需能力的缺失
    for (const auto& req_capability : requirement.required_capabilities) {
        if (!hasSpecificCapability(profile, req_capability)) {
            result.missing_capabilities.push_back(req_capability);
            result.incompatibility_reasons.push_back("Missing required capability: " + req_capability);
        }
    }
    
    // 分析数值参数的不匹配
    for (const auto& numeric_req : requirement.numeric_ranges) {
        const std::string& param_name = numeric_req.first;
        const auto& range = numeric_req.second;
        
        float robot_value = getRobotNumericParameter(profile, param_name);
        if (robot_value >= 0.0f) {
            if (robot_value < range.first) {
                result.incompatibility_reasons.push_back(
                    param_name + " too low: " + std::to_string(robot_value) + 
                    " < " + std::to_string(range.first));
            } else if (robot_value > range.second) {
                result.incompatibility_reasons.push_back(
                    param_name + " too high: " + std::to_string(robot_value) + 
                    " > " + std::to_string(range.second));
            }
        }
    }
}

void CapabilityManager::generateImprovementRecommendations(
    const RobotCapabilityProfile& profile,
    const CapabilityRequirement& requirement,
    CapabilityMatchResult& result) const {
    
    // 为缺失的能力生成建议
    for (const auto& missing_cap : result.missing_capabilities) {
        if (missing_cap == "lidar_sensor") {
            result.recommendations.push_back("Consider adding LiDAR sensor for better navigation");
        } else if (missing_cap == "high_speed_motion") {
            result.recommendations.push_back("Upgrade motion control system for higher speeds");
        } else if (missing_cap == "heavy_payload") {
            result.recommendations.push_back("Consider load distribution or multiple robot collaboration");
        } else {
            result.recommendations.push_back("Add support for capability: " + missing_cap);
        }
    }
    
    // 为数值参数不匹配生成建议
    for (const auto& numeric_req : requirement.numeric_ranges) {
        const std::string& param_name = numeric_req.first;
        const auto& range = numeric_req.second;
        
        float robot_value = getRobotNumericParameter(profile, param_name);
        if (robot_value >= 0.0f && (robot_value < range.first || robot_value > range.second)) {
            result.recommendations.push_back(
                "Optimize " + param_name + " to be within range [" + 
                std::to_string(range.first) + ", " + std::to_string(range.second) + "]");
        }
    }
}

float CapabilityManager::calculateCollaborativeScore(
    const std::vector<RobotCapabilityProfile>& profiles,
    const CapabilityRequirement& requirement,
    const std::vector<float>& individual_scores) const {
    
    if (profiles.empty() || individual_scores.empty()) {
        return 0.0f;
    }
    
    // 策略1: 平均评分 (40%)
    float average_score = 0.0f;
    for (float score : individual_scores) {
        average_score += score;
    }
    average_score /= individual_scores.size();
    
    // 策略2: 最佳评分 (30%)
    float best_score = *std::max_element(individual_scores.begin(), individual_scores.end());
    
    // 策略3: 互补性加成 (30%)
    float complementarity_bonus = calculateComplementarityBonus(profiles, requirement);
    
    // 综合评分
    return 0.4f * average_score + 0.3f * best_score + 0.3f * complementarity_bonus;
}

float CapabilityManager::calculateComplementarityBonus(
    const std::vector<RobotCapabilityProfile>& profiles,
    const CapabilityRequirement& requirement) const {
    
    // 检查不同机器人是否能互补缺失的能力
    std::set<std::string> collective_capabilities;
    
    // 收集所有机器人的能力
    for (const auto& profile : profiles) {
        for (const auto& capability : profile.capabilities) {
            collective_capabilities.insert(capability.capability_name);
        }
    }
    
    // 计算需求覆盖率
    int covered_requirements = 0;
    for (const auto& req_capability : requirement.required_capabilities) {
        if (collective_capabilities.find(req_capability) != collective_capabilities.end()) {
            covered_requirements++;
        }
    }
    
    if (requirement.required_capabilities.empty()) {
        return 1.0f;
    }
    
    float coverage_ratio = static_cast<float>(covered_requirements) / requirement.required_capabilities.size();
    
    // 多样性加成：机器人类型越多样，加成越高
    std::set<RobotType> unique_types;
    for (const auto& profile : profiles) {
        unique_types.insert(profile.robot_type);
    }
    
    float diversity_bonus = std::min(1.0f, static_cast<float>(unique_types.size()) / 3.0f);
    
    return coverage_ratio * (0.7f + 0.3f * diversity_bonus);
}

void CapabilityManager::analyzeCapabilityComplementarity(
    const std::vector<RobotCapabilityProfile>& profiles,
    const CapabilityRequirement& requirement,
    CapabilityMatchResult& result) const {
    
    // 分析哪些机器人提供了哪些关键能力
    std::map<std::string, std::vector<RobotType>> capability_providers;
    
    for (const auto& req_capability : requirement.required_capabilities) {
        for (const auto& profile : profiles) {
            if (hasSpecificCapability(profile, req_capability)) {
                capability_providers[req_capability].push_back(profile.robot_type);
            }
        }
    }
    
    // 识别单点故障和冗余
    for (const auto& cap_provider : capability_providers) {
        const std::string& capability = cap_provider.first;
        const auto& providers = cap_provider.second;
        
        if (providers.empty()) {
            result.parameter_conflicts["critical_gap"] = 
                "No robot provides capability: " + capability;
        } else if (providers.size() == 1) {
            result.parameter_conflicts["single_point_failure"] = 
                "Only one robot provides: " + capability;
        } else {
            result.parameter_conflicts["redundancy"] = 
                capability + " provided by " + std::to_string(providers.size()) + " robots";
        }
    }
}

void CapabilityManager::setLastError(const std::string& error_message) const {
    last_error_ = error_message;
}

} // namespace capability_manager
} // namespace robot_factory