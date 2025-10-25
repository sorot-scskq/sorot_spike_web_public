/**
 * YAML設定ファイル読み込みシステム
 * コメント対応の設定管理
 */

export class YAMLConfigLoader {
    constructor() {
        this.yamlParser = null;
        this.loadYAMLParser();
    }

    /**
     * YAMLパーサーを動的に読み込み
     */
    async loadYAMLParser() {
        try {
            // js-yamlライブラリを動的に読み込み
            const script = document.createElement('script');
            script.src = 'https://cdnjs.cloudflare.com/ajax/libs/js-yaml/4.1.0/js-yaml.min.js';
            script.onload = () => {
                this.yamlParser = window.jsyaml;
                console.log('YAMLパーサーが読み込まれました');
            };
            document.head.appendChild(script);
        } catch (error) {
            console.error('YAMLパーサーの読み込みに失敗しました:', error);
        }
    }

    /**
     * YAMLファイルを読み込んでオブジェクトに変換
     * @param {string} yamlText - YAML形式のテキスト
     * @returns {Object} パースされたオブジェクト
     */
    parseYAML(yamlText) {
        if (!this.yamlParser) {
            throw new Error('YAMLパーサーが読み込まれていません');
        }

        try {
            return this.yamlParser.load(yamlText);
        } catch (error) {
            console.error('YAMLパースエラー:', error);
            throw new Error(`YAML設定ファイルのパースに失敗しました: ${error.message}`);
        }
    }

    /**
     * YAML設定ファイルを非同期で読み込み
     * @param {string} filePath - ファイルパス
     * @returns {Promise<Object>} 設定オブジェクト
     */
    async loadYAMLConfig(filePath) {
        try {
            const response = await fetch(filePath);
            if (!response.ok) {
                throw new Error(`HTTP error! status: ${response.status}`);
            }
            const yamlText = await response.text();
            return this.parseYAML(yamlText);
        } catch (error) {
            console.error(`YAML設定ファイル読み込みエラー (${filePath}):`, error);
            throw error;
        }
    }

    /**
     * 複数のYAML設定ファイルを一括読み込み
     * @param {string[]} filePaths - ファイルパスの配列
     * @returns {Promise<Object>} 統合された設定オブジェクト
     */
    async loadYAMLConfigs(filePaths) {
        const configs = {};
        
        for (const filePath of filePaths) {
            try {
                const fileName = filePath.split('/').pop().replace('.yaml', '').replace('.yml', '');
                configs[fileName] = await this.loadYAMLConfig(filePath);
            } catch (error) {
                console.warn(`YAML設定ファイル ${filePath} の読み込みをスキップしました:`, error);
            }
        }
        
        return configs;
    }
}

/**
 * 複数年分のYAML設定管理クラス
 */
export class YAMLConfigManager {
    constructor() {
        this.loader = new YAMLConfigLoader();
        this.currentYear = "2023";
        this.configs = new Map();
        this.isLoaded = false;
    }

    /**
     * 初期化処理
     */
    async initialize() {
        try {
            // YAMLパーサーの読み込み完了を待つ
            await this.waitForYAMLParser();
            await this.loadAllConfigs();
            this.isLoaded = true;
            console.log('YAML設定の読み込みが完了しました');
        } catch (error) {
            console.error('YAML設定の読み込みに失敗しました:', error);
            throw error;
        }
    }

    /**
     * YAMLパーサーの読み込み完了を待つ
     */
    async waitForYAMLParser() {
        return new Promise((resolve) => {
            const checkParser = () => {
                if (this.loader.yamlParser) {
                    resolve();
                } else {
                    setTimeout(checkParser, 100);
                }
            };
            checkParser();
        });
    }

    /**
     * 全年度の設定を読み込み
     */
    async loadAllConfigs() {
        const years = ["2023","2024","2025","Test"]; // 対応年度を追加
        
        for (const year of years) {
            try {
                const config = await this.loadYearConfig(year);
                this.configs.set(year, config);
                console.log(`${year}年のYAML設定を読み込みました`);
            } catch (error) {
                console.warn(`${year}年のYAML設定読み込みをスキップしました:`, error);
            }
        }
    }

    /**
     * 指定年度の設定を読み込み
     * @param {number} year - 年度
     * @returns {Object} 設定オブジェクト
     */
    async loadYearConfig(year) {
        const configFiles = [
            `config/years/${year}/robot.yaml`,
            `config/years/${year}/course.yaml`,
            // `config/years/${year}/sensors.yaml`,
            // `config/years/${year}/rules.yaml`,
        ];

        const configs = await this.loader.loadYAMLConfigs(configFiles);
        
        return {
            year: year,
            robot: configs.robot || {},
            course: configs.course || {},
            sensors: configs.sensors || {},
            rules: configs.rules || {}
        };
    }

    /**
     * 現在の年度の設定を取得
     * @returns {Object} 設定オブジェクト
     */
    getCurrentConfig() {
        return this.getConfig(this.currentYear);
    }

    /**
     * 指定年度の設定を取得
     * @param {number} year - 年度
     * @returns {Object} 設定オブジェクト
     */
    getConfig(year) {
        if (!this.isLoaded) {
            throw new Error('設定がまだ読み込まれていません。initialize()を先に呼び出してください。');
        }

        const config = this.configs.get(year);
        config.imgpath = this.getPath(year) + config.course.image.filename;
        if (!config) {
            throw new Error(`${year}年の設定が見つかりません`);
        }

        return config;
    }

    /**
     * 現在の年度を設定
     * @param {number} year - 年度
     */
    setCurrentYear(year) {
        if (!this.configs.has(year)) {
            throw new Error(`${year}年の設定が読み込まれていません`);
        }
        
        this.currentYear = year;
        console.log(`現在の年度を${year}年に設定しました`);
    }

    /**
     * 利用可能な年度のリストを取得
     * @returns {number[]} 年度の配列
     */
    getAvailableYears() {
        return Array.from(this.configs.keys()).sort();
    }

    /**
     * パスを取得
     * @param {*} year 
     * @returns 
     */
    getPath(year) {
        return `/config/years/${year}/`
    }
}

// グローバルに登録
window.YAMLConfigManager = YAMLConfigManager;
window.YAMLConfigLoader = YAMLConfigLoader;
