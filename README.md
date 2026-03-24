以下にアクセスしてください。

https://sorot-scskq.github.io/sorot_spike_web_public/

このサイトを最新版に保つ方法

## リポジトリに必要な設定
1.SPIKE_PUBLIC_DEPLOY_TOKENの設定

https://github.com/sorot-scskq/sorot_spike_web にシークレット SPIKE_PUBLIC_DEPLOY_TOKEN を登録し、sorot_spike_web_public へ contents: write できる PAT（classic）か、該当リポのみの fine-grained PAT を使ってください。未設定のときはワークフローがエラー終了します。

2.sorot_spike_webのメインリポジトリにpush

https://github.com/sorot-scskq/sorot_spike_web のmainブランチをプッシュすることでActionsが実行されます。
