from openai import OpenAI

class DeepSeek:
    def __init__(self, api_key:str = "sk-530fb7fc0a0b4bab81486c0b005e681f", base_url: str = "https://api.deepseek.com/v1", default_sys_prompt: str = ""):
        """
        初始化DeepSeek客户端
        :param api_key: DeepSeek API密钥
        :param base_url: 兼容OpenAI接口地址
        :param default_sys_prompt: 默认系统提示词
        """
        self.api_key = api_key
        self.base_url = base_url
        self.default_sys_prompt = default_sys_prompt
        # 实例化OpenAI兼容客户端
        self.client = OpenAI(
            api_key=self.api_key,
            base_url=self.base_url
        )

    def set_default_prompt(self, new_prompt: str):
        """对外接口：修改全局默认系统提示词"""
        self.default_sys_prompt = new_prompt

    def get_reply(self, user_text: str, custom_system_prompt: str = None) -> str:
        """
        对外核心接口：获取模型回复
        :param user_text: 用户输入文本（语音识别文字）
        :param custom_system_prompt: 单次对话自定义提示词，优先级高于默认
        :return: 模型回答字符串
        """
        # 优先使用单次传入的自定义提示词，无则使用全局默认
        sys_prompt = custom_system_prompt if custom_system_prompt is not None else self.default_sys_prompt
        try:
            completion = self.client.chat.completions.create(
                model="deepseek-chat",
                messages=[
                    {"role": "system", "content": sys_prompt},
                    {"role": "user", "content": user_text}
                ],
                temperature=0.7,
                max_tokens=128
            )
            ans = completion.choices[0].message.content.strip()
            return ans
        except Exception as e:
            return f"大模型调用失败：{str(e)}"


# ========== 全局实例化配置（业务使用入口） ==========
if __name__ == "__main__":
    # 初始化密钥、地址、默认提示词
    DEFAULT_SYS_PROMPT = "你是智能助手，回答简短精炼，不超过30个字；运动指令直接输出关键词"

    # 创建客户端实例
    ds_client = DeepSeekClient(
        default_sys_prompt=DEFAULT_SYS_PROMPT
    )

    # 测试1：使用默认提示词调用
    res1 = ds_client.get_reply("前进")
    print("测试1-默认提示词回复：", res1)

    # 测试2：单次调用传入自定义提示词（仅本次生效）
    custom_prompt = "你是小车控制助手，用一句话描述动作，不要只输出关键词"
    res2 = ds_client.get_reply("后退", custom_system_prompt=custom_prompt)
    print("测试2-单次自定义提示词回复：", res2)

    # 测试3：修改全局默认提示词，后续所有调用生效
    ds_client.set_default_prompt("你是智能小车，动作指令用完整短句回答")
    res3 = ds_client.get_reply("左转")
    print("测试3-修改全局默认提示词后回复：", res3)