module.exports = {
  env: {
    commonjs: true,
    es6: true,
    browser: true,
  },
  extends: 'airbnb-base',
  globals: {
    Atomics: 'readonly',
    SharedArrayBuffer: 'readonly',
  },
  parserOptions: {
    ecmaVersion: 2018,
  },
  rules: {
    'no-console':'off',
    'no-plusplus': ["error", { "allowForLoopAfterthoughts": true }],
    'no-restricted-syntax':'off',
  },
};
