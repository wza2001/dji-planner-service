# Plan: 爬取 DJI WPML 文档并生成 Markdown

## Stage 1 — 抓取
- web_open_url 抓取两个页面：
  1. template-kml.html (WPML 模板 KML)
  2. waylines-wpml.html (航线 WPML)
- 若 web_open_url 返回内容不完整（JS 渲染），改用 browser_visit 或 curl 抓取原始 HTML。

## Stage 2 — 转换
- 将内容无损转换为 Markdown，保留所有标签、属性、枚举值、说明表格。
- 每个页面各生成一个 md 文件，保存至 /mnt/agents/output/。

## Stage 3 — 校验
- 检查 md 文件完整性（标签数量、表格完整性），确认无信息丢失。
