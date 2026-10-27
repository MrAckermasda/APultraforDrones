# 当前代码快照：GitHub 上传后续步骤

日期：2026-10-27

## 当前进度

已经初始化本地 Git 仓库，分支为 `main`；已编写 `.gitignore`、README 和本机配置示例。用户随后指定远端 `https://github.com/MrAckermasda/APultraforDrones.git`、邮箱 `lunyingliu@126.com` 和提交说明 `first commit`。提交姓名使用 `MrAckermasda`。下文保留通用操作步骤，执行前先用 `git log -1` 和 `git remote -v` 检查实际进度，避免重复创建提交或远端。

本次访问 GitHub 出现 TLS 证书链不受信任。若本机终端网络正常，可在本地提交完成后执行 `git push -u origin main`。若同样失败，先检查本机网络代理及受信任 CA 配置，不要通过 `http.sslVerify=false` 绕过验证。

已经保存的本地归档：

```text
.migration-backup/github-snapshot-2026-10-27/source-snapshot.tar.gz
```

这个归档是当时暂存代码的快照，不会随着之后的修改自动更新，也不包含本文。它不是整台设备的备份；忽略的模型、本机配置、原 ARM 共享库等仍留在工作区中。

## 1. 准备 GitHub 仓库

登录自己的 GitHub，在页面中创建仓库：

- 名称可以使用 `armpi-ultra-pc`。
- 建议先选择 Private（私有），后续有需要再公开。
- 创建空仓库，不勾选自动生成 README、`.gitignore` 或 License；本地已有相关内容，空仓库便于首次推送。
- 复制页面提供的 HTTPS 或 SSH 仓库地址。

如果已经有目标仓库，直接使用它，并先确认它是否为空。下文的直接推送流程以空仓库为前提；已有提交时先检查远端历史，不要使用强制推送覆盖。

## 2. 配置提交身份

在终端执行，将占位文字替换为自己的信息：

```bash
cd /home/lyl/ros2_ws
git config --local user.name "你的提交姓名"
git config --local user.email "你的提交邮箱"
```

这两项只是 Git 提交作者信息，不是登录密码。若不希望公开个人邮箱，可以使用 GitHub 账户邮箱设置中显示的 noreply 地址，直接复制完整地址，不要猜测其格式。`--local` 只配置此工作区。

## 3. 完成本地提交

```bash
cd /home/lyl/ros2_ws
git add -A
git status --short
git diff --cached --stat
```

检查暂存清单：应包含源码、文档、动作组、测试和配置示例，不应包含根目录 `build/`、`install/`、`log/`、`.migration-backup/`、私有 `config/pc.env`、模型权重或实际密钥。若之后新增了配置文件，也需要检查其中是否含密码或令牌。

确认清单后提交：

```bash
git commit -m "Save ArmPi PC port, gamepad controls and offline Python FK/IK"
git log -1 --oneline
```

当前 Python FK/IK 是待实机校准的离线原型，提交说明不要写成已部署在线运动学控制。

## 4. 关联远端并上传

先查看是否已有远端：

```bash
git remote -v
```

目前尚未配置远端。如果输出为空，把下面的占位文字替换成刚才复制的完整仓库地址：

```bash
git remote add origin '替换成完整的GitHub仓库地址'
git push -u origin main
```

如果已经存在 `origin`，先确认它就是目标仓库，不要重复添加或随意覆盖。

首次 push 可能要求认证：HTTPS 使用本机配置的凭据管理器或 GitHub 访问令牌，SSH 使用已关联账户的 SSH 密钥。不要把令牌写入仓库 URL、源码、文档或聊天消息；普通账户密码不用于 Git HTTPS 推送。按本机认证提示完成登录即可。

若看到权限不足、认证失败或远端已有提交，保留完整错误信息（去除任何凭据），再针对错误处理，不要直接 `--force`。

## 5. 验证上传

```bash
git status --short
git rev-parse HEAD
git ls-remote origin refs/heads/main
```

确认本地 `HEAD` 与远端 `main` 的提交号一致，并打开 GitHub 仓库检查源码和文档。`git status --short` 无输出表示没有待提交的变更（被忽略的本机文件不会显示）。

## 可以让我继续完成的内容

提供以下信息即可继续配置、提交并推送：

1. 目标仓库的完整地址。
2. Git 提交姓名。
3. Git 提交邮箱，可用 GitHub noreply 邮箱。

账户认证在你本机完成，不需要把密码、令牌或 SSH 私钥发给我。若尚未创建仓库，先完成第一步。此前保存在本机的代码快照不会因等待这些信息而丢失。
