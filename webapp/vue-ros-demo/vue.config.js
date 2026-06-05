const { defineConfig } = require('@vue/cli-service');
module.exports = defineConfig({
  transpileDependencies: true,
  lintOnSave: false,
  chainWebpack: config => {
    config.plugin('html').tap(args => {
      args[0].title = 'FEAST_System';
      return args;
    });
  },
  devServer: {
    client: {
      overlay: false,  // 禁用开发模式下的错误覆盖显示
    }
  }
});
