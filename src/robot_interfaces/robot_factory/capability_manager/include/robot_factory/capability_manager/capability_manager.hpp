/**
 * @file capability_manager.hpp
 * @brief 机器人能力管理器 - 管理和匹配不同机器人的能力
 * @author Claude Code
 * @date 2024
 */

#ifndef ROBOT_FACTORY__CAPABILITY_MANAGER__CAPABILITY_MANAGER_HPP_
#define ROBOT_FACTORY__CAPABILITY_MANAGER__CAPABILITY_MANAGER_HPP_

#include <string>
#include <vector>
#include <map>
#include <memory>

// 导入共同的类型定义
#include "robot_factory/capability_manager/capability_types.hpp"

namespace robot_factory {
namespace capability_manager {

/**
 * @brief 单个能力定义
 */
struct CapabilityDefinition {
    std::string capability_id;          ///< 能力唯一标识
    std::string capability_name;        ///< 能力名称
    CapabilityType type;               ///< 能力类型
    CapabilityLevel level;             ///< 能力级别
    std::string description;           ///< 能力描述
    
    // 能力参数和限制
    std::map<std::string, float> numeric_parameters;  ///< 数值参数
    std::map<std::string, bool> boolean_parameters;   ///< 布尔参数
    std::map<std::string, std::string> string_parameters; ///< 字符串参数
    
    // 依赖关系
    std::vector<std::string> required_capabilities;   ///< 依赖的其他能力
    std::vector<std::string> optional_capabilities;   ///< 可选的其他能力
    
    // 版本和兼容性
    std::string version = "1.0.0";                   ///< 能力版本
    std::vector<std::string> compatible_versions;    ///< 兼容的版本列表
    
    CapabilityDefinition() = default;
    CapabilityDefinition(const std::string& id, const std::string& name, 
                        CapabilityType t, CapabilityLevel l)
        : capability_id(id), capability_name(name), type(t), level(l) {}
};

/**
 * @brief 机器人能力配置文件
 */
struct RobotCapabilityProfile {
    RobotType robot_type;              ///< 机器人类型
    std::string robot_name;            ///< 机器人名称
    std::string robot_model;           ///< 机器人型号
    std::string profile_version = "1.0.0"; ///< 配置文件版本
    
    // 基础物理属性
    struct {
        float length = 0.0f;           ///< 长度 (m)
        float width = 0.0f;            ///< 宽度 (m)
        float height = 0.0f;           ///< 高度 (m)
        float weight = 0.0f;           ///< 重量 (kg)
        float max_payload = 0.0f;      ///< 最大负载 (kg)
    } physical_properties;
    
    // 支持的能力列表
    std::vector<CapabilityDefinition> capabilities;
    
    // 能力组合和配置
    std::map<std::string, std::vector<std::string>> capability_groups; ///< 能力分组
    std::map<std::string, std::string> default_configurations;         ///< 默认配置
    
    // 限制和约束
    struct {
        float max_operating_temperature = 60.0f;  ///< 最高工作温度 (°C)
        float min_operating_temperature = -20.0f; ///< 最低工作温度 (°C)
        float max_humidity = 95.0f;              ///< 最大湿度 (%)
        float max_altitude = 3000.0f;            ///< 最大海拔 (m)
    } environmental_limits;
    
    RobotCapabilityProfile() = default;
    RobotCapabilityProfile(RobotType type, const std::string& name, const std::string& model)
        : robot_type(type), robot_name(name), robot_model(model) {}
};

/**
 * @brief 能力匹配结果
 */
struct CapabilityMatchResult {
    bool is_compatible = false;        ///< 是否兼容
    float compatibility_score = 0.0f;  ///< 兼容性评分 (0.0-1.0)
    
    // 匹配详情
    std::vector<std::string> matched_capabilities;    ///< 匹配的能力
    std::vector<std::string> missing_capabilities;    ///< 缺失的能力
    std::vector<std::string> exceeded_capabilities;   ///< 超出需求的能力
    
    // 不兼容原因
    std::vector<std::string> incompatibility_reasons; ///< 不兼容原因列表
    std::map<std::string, std::string> parameter_conflicts; ///< 参数冲突
    
    // 建议
    std::vector<std::string> recommendations;         ///< 改进建议
    
    CapabilityMatchResult() = default;
    CapabilityMatchResult(bool compatible, float score) 
        : is_compatible(compatible), compatibility_score(score) {}
};

/**
 * @brief 能力需求定义
 */
struct CapabilityRequirement {
    std::string requirement_id;       ///< 需求唯一标识
    std::string requirement_name;     ///< 需求名称
    std::string description;          ///< 需求描述
    
    // 必需能力
    std::vector<std::string> required_capabilities;   ///< 必需的能力ID列表
    std::vector<std::string> optional_capabilities;   ///< 可选的能力ID列表
    
    // 能力参数要求
    std::map<std::string, std::pair<float, float>> numeric_ranges; ///< 数值范围要求
    std::map<std::string, bool> boolean_requirements;              ///< 布尔要求
    std::map<std::string, std::vector<std::string>> string_options; ///< 字符串选项
    
    // 优先级和权重
    std::map<std::string, float> capability_weights;  ///< 各能力的权重
    float minimum_score = 0.7f;                      ///< 最低兼容性评分
    
    CapabilityRequirement() = default;
    CapabilityRequirement(const std::string& id, const std::string& name)
        : requirement_id(id), requirement_name(name) {}
};

/**
 * @brief 机器人能力管理器类
 * 
 * 该类负责：
 * 1. 管理不同机器人的能力配置文件
 * 2. 提供能力查询和比较功能
 * 3. 执行能力匹配和兼容性检查
 * 4. 支持能力配置的动态加载和更新
 */
class CapabilityManager {
public:
    CapabilityManager();
    virtual ~CapabilityManager() = default;
    
    // ============= 初始化和配置 =============
    
    /**
     * @brief 初始化能力管理器
     * @param config_directory 配置文件目录
     * @return true if successful, false otherwise
     */
    bool initialize(const std::string& config_directory = "");
    
    /**
     * @brief 关闭能力管理器
     * @return true if successful, false otherwise
     */
    bool shutdown();
    
    /**
     * @brief 检查管理器是否已初始化
     * @return true if initialized, false otherwise
     */
    bool isInitialized() const { return initialized_; }
    
    // ============= 能力配置文件管理 =============
    
    /**
     * @brief 加载机器人能力配置文件
     * @param file_path 配置文件路径
     * @return true if successful, false otherwise
     */
    bool loadCapabilityProfile(const std::string& file_path);
    
    /**
     * @brief 注册机器人能力配置
     * @param profile 能力配置文件
     * @return true if successful, false otherwise
     */
    bool registerCapabilityProfile(const RobotCapabilityProfile& profile);
    
    /**
     * @brief 获取机器人能力配置
     * @param robot_type 机器人类型
     * @return 能力配置文件指针，未找到时返回nullptr
     */
    std::shared_ptr<RobotCapabilityProfile> getCapabilityProfile(RobotType robot_type) const;
    
    /**
     * @brief 获取所有已注册的机器人类型
     * @return 机器人类型列表
     */
    std::vector<RobotType> getRegisteredRobotTypes() const;
    
    /**
     * @brief 更新机器人能力配置
     * @param robot_type 机器人类型
     * @param profile 新的能力配置
     * @return true if successful, false otherwise
     */
    bool updateCapabilityProfile(RobotType robot_type, const RobotCapabilityProfile& profile);
    
    // ============= 能力查询和比较 =============
    
    /**
     * @brief 检查机器人是否支持特定能力
     * @param robot_type 机器人类型
     * @param capability_id 能力ID
     * @return true if supported, false otherwise
     */
    bool hasCapability(RobotType robot_type, const std::string& capability_id) const;
    
    /**
     * @brief 获取机器人的特定能力定义
     * @param robot_type 机器人类型
     * @param capability_id 能力ID
     * @return 能力定义指针，未找到时返回nullptr
     */
    std::shared_ptr<CapabilityDefinition> getCapabilityDefinition(
        RobotType robot_type, const std::string& capability_id) const;
    
    /**
     * @brief 获取机器人的所有能力
     * @param robot_type 机器人类型
     * @param capability_type 能力类型过滤 (可选)
     * @return 能力定义列表
     */
    std::vector<CapabilityDefinition> getRobotCapabilities(
        RobotType robot_type, CapabilityType capability_type = CapabilityType::CUSTOM) const;
    
    /**
     * @brief 比较两个机器人的能力
     * @param robot_type1 机器人类型1
     * @param robot_type2 机器人类型2
     * @return 比较结果 (JSON格式字符串)
     */
    std::string compareRobotCapabilities(RobotType robot_type1, RobotType robot_type2) const;
    
    // ============= 能力匹配和兼容性检查 =============
    
    /**
     * @brief 检查机器人是否满足能力需求
     * @param robot_type 机器人类型
     * @param requirement 能力需求
     * @return 匹配结果
     */
    CapabilityMatchResult matchCapabilityRequirement(
        RobotType robot_type, const CapabilityRequirement& requirement) const;
    
    /**
     * @brief 查找满足需求的最佳机器人
     * @param requirement 能力需求
     * @return 机器人类型和匹配结果的配对列表，按兼容性评分排序
     */
    std::vector<std::pair<RobotType, CapabilityMatchResult>> findBestMatchingRobots(
        const CapabilityRequirement& requirement) const;
    
    /**
     * @brief 检查多个机器人的协同能力
     * @param robot_types 机器人类型列表
     * @param requirement 协同能力需求
     * @return 协同匹配结果
     */
    CapabilityMatchResult matchCollaborativeCapability(
        const std::vector<RobotType>& robot_types, 
        const CapabilityRequirement& requirement) const;
    
    // ============= 能力需求管理 =============
    
    /**
     * @brief 注册能力需求模板
     * @param requirement 能力需求
     * @return true if successful, false otherwise
     */
    bool registerCapabilityRequirement(const CapabilityRequirement& requirement);
    
    /**
     * @brief 获取能力需求模板
     * @param requirement_id 需求ID
     * @return 能力需求指针，未找到时返回nullptr
     */
    std::shared_ptr<CapabilityRequirement> getCapabilityRequirement(
        const std::string& requirement_id) const;
    
    /**
     * @brief 获取所有已注册的能力需求
     * @return 能力需求列表
     */
    std::vector<CapabilityRequirement> getAllCapabilityRequirements() const;
    
    // ============= Go2特定能力配置 =============
    
    /**
     * @brief 创建Go2默认能力配置
     * @return Go2能力配置文件
     */
    static RobotCapabilityProfile createGo2DefaultProfile();
    
    /**
     * @brief 创建导航系统的能力需求
     * @return 导航能力需求
     */
    static CapabilityRequirement createNavigationRequirement();
    
    /**
     * @brief 创建自主充电的能力需求
     * @return 自主充电能力需求
     */
    static CapabilityRequirement createAutoChargingRequirement();
    
    // ============= 配置文件I/O =============
    
    /**
     * @brief 导出能力配置到文件
     * @param robot_type 机器人类型
     * @param file_path 导出文件路径
     * @param format 导出格式 ("json", "yaml")
     * @return true if successful, false otherwise
     */
    bool exportCapabilityProfile(RobotType robot_type, const std::string& file_path,
                                const std::string& format = "json") const;
    
    /**
     * @brief 导入能力配置从文件
     * @param file_path 配置文件路径
     * @param format 文件格式 ("json", "yaml", "auto")
     * @return true if successful, false otherwise
     */
    bool importCapabilityProfile(const std::string& file_path,
                                const std::string& format = "auto");
    
    /**
     * @brief 生成能力配置模板
     * @param robot_type 机器人类型
     * @param file_path 模板文件路径
     * @return true if successful, false otherwise
     */
    bool generateCapabilityTemplate(RobotType robot_type, const std::string& file_path) const;
    
    // ============= 工具方法 =============
    
    /**
     * @brief 将机器人类型转换为字符串
     * @param robot_type 机器人类型
     * @return 类型字符串
     */
    static std::string robotTypeToString(RobotType robot_type);
    
    /**
     * @brief 将字符串转换为机器人类型
     * @param type_string 类型字符串
     * @return 机器人类型
     */
    static RobotType stringToRobotType(const std::string& type_string);
    
    /**
     * @brief 获取能力管理器版本
     * @return 版本字符串
     */
    std::string getVersion() const { return "1.0.0"; }
    
    /**
     * @brief 获取最后的错误信息
     * @return 错误信息
     */
    std::string getLastError() const { return last_error_; }

private:
    // ============= 私有成员变量 =============
    
    bool initialized_ = false;                       ///< 是否已初始化
    std::string config_directory_;                   ///< 配置文件目录
    
    // 能力配置存储
    std::map<RobotType, std::shared_ptr<RobotCapabilityProfile>> capability_profiles_;
    std::map<std::string, std::shared_ptr<CapabilityRequirement>> capability_requirements_;
    
    // 错误信息
    mutable std::string last_error_;
    
    // ============= 私有辅助方法 =============
    
    /**
     * @brief 加载默认能力配置
     */
    void loadDefaultProfiles();
    
    /**
     * @brief 验证能力配置文件
     * @param profile 能力配置文件
     * @return true if valid, false otherwise
     */
    bool validateCapabilityProfile(const RobotCapabilityProfile& profile) const;
    
    /**
     * @brief 计算兼容性评分
     * @param profile 机器人能力配置
     * @param requirement 能力需求
     * @return 兼容性评分 (0.0-1.0)
     */
    float calculateCompatibilityScore(const RobotCapabilityProfile& profile,
                                     const CapabilityRequirement& requirement) const;
    
    /**
     * @brief 设置最后的错误信息
     * @param error_message 错误信息
     */
    void setLastError(const std::string& error_message) const;
    
    // ============= 新增的私有辅助方法 =============
    
    /**
     * @brief 执行详细的能力分析
     */
    void performDetailedCapabilityAnalysis(
        const RobotCapabilityProfile& profile,
        const CapabilityRequirement& requirement,
        CapabilityMatchResult& result) const;
    
    /**
     * @brief 生成改进建议
     */
    void generateImprovementRecommendations(
        const RobotCapabilityProfile& profile,
        const CapabilityRequirement& requirement,
        CapabilityMatchResult& result) const;
    
    /**
     * @brief 计算协作评分
     */
    float calculateCollaborativeScore(
        const std::vector<RobotCapabilityProfile>& profiles,
        const CapabilityRequirement& requirement,
        const std::vector<float>& individual_scores) const;
    
    /**
     * @brief 分析能力互补性
     */
    void analyzeCapabilityComplementarity(
        const std::vector<RobotCapabilityProfile>& profiles,
        const CapabilityRequirement& requirement,
        CapabilityMatchResult& result) const;
    
    /**
     * @brief 评估必需能力
     */
    float evaluateRequiredCapabilities(
        const RobotCapabilityProfile& profile,
        const CapabilityRequirement& requirement) const;
    
    /**
     * @brief 评估数值要求
     */
    float evaluateNumericRequirements(
        const RobotCapabilityProfile& profile,
        const CapabilityRequirement& requirement) const;
    
    /**
     * @brief 评估可选能力
     */
    float evaluateOptionalCapabilities(
        const RobotCapabilityProfile& profile,
        const CapabilityRequirement& requirement) const;
    
    /**
     * @brief 检查特定能力
     */
    bool hasSpecificCapability(
        const RobotCapabilityProfile& profile,
        const std::string& capability_name) const;
    
    /**
     * @brief 获取能力权重
     */
    float getCapabilityWeight(
        const CapabilityRequirement& requirement,
        const std::string& capability_name) const;
    
    /**
     * @brief 获取机器人数值参数
     */
    float getRobotNumericParameter(
        const RobotCapabilityProfile& profile,
        const std::string& parameter_name) const;
    
    /**
     * @brief 计算数值评分
     */
    float calculateNumericScore(
        float robot_value, float min_required, float max_required) const;
    
    /**
     * @brief 计算互补性加成
     */
    float calculateComplementarityBonus(
        const std::vector<RobotCapabilityProfile>& profiles,
        const CapabilityRequirement& requirement) const;
};

/**
 * @brief 机器人能力管理器智能指针类型定义
 */
using CapabilityManagerPtr = std::shared_ptr<CapabilityManager>;

} // namespace capability_manager
} // namespace robot_factory

#endif // ROBOT_FACTORY__CAPABILITY_MANAGER__CAPABILITY_MANAGER_HPP_