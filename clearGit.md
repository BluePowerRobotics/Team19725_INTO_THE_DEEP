当 Git 未正确忽略 `.gitignore` 中指定的文件时，通常是因为文件已被跟踪或忽略规则配置有误。以下是分步解决方案：

### 1. 检查 `.gitignore` 规则
- **语法验证**：确保规则语法正确，例如使用 `*.log` 忽略所有日志文件，`/build/` 忽略根目录下的 `build` 文件夹。
- **路径匹配**：确认路径是否准确。若文件在子目录中，规则应为 `subdir/file`，而非绝对路径。

### 2. 移除已跟踪的文件
若文件已被 Git 跟踪（如之前提交过），需从索引中移除（保留本地文件）：
```bash
git rm --cached <file_path>       # 移除单个文件
git rm --cached -r <directory>   # 递归移除目录
```
例如：
```bash
git rm --cached logs/debug.log
```

### 3. 提交变更
将 `.gitignore` 和索引变更提交到仓库：
```bash
git add .gitignore
git commit -m "Update .gitignore and remove tracked files"
```

### 4. 验证忽略规则
使用以下命令检查忽略规则是否生效：
```bash
git check-ignore -v <file_path>
```
输出会显示匹配的忽略规则，若无输出则说明未被忽略。

### 5. 清除缓存（可选）
若规则正确但未生效，尝试清理 Git 缓存：
```bash
git rm -r --cached .  # 清除所有缓存
git add .
git commit -m "Reset git cache"
```

### 6. 特殊案例处理
- **全局忽略**：若需全局忽略某类文件（如编辑器临时文件），配置全局 `.gitignore`：
  ```bash
  git config --global core.excludesfile ~/.global_gitignore
  ```
- **强制添加**：避免使用 `git add -f` 强制添加被忽略文件，除非必要。

### 示例场景
**问题**：已提交的 `node_modules` 未被忽略。
**解决**：
```bash
# 从索引中移除 node_modules
git rm --cached -r node_modules

# 确认 .gitignore 包含 node_modules/
echo "node_modules/" >> .gitignore

# 提交更改
git add .gitignore
git commit -m "Ignore node_modules"
```

### 注意事项
- **协作影响**：若文件已被他人提交，需同步更新仓库并告知团队执行类似操作。
- **规则优先级**：子目录的 `.gitignore` 会覆盖父级规则，注意层级影响。

通过以上步骤，可确保 `.gitignore` 正确生效，避免不需要的文件被跟踪。