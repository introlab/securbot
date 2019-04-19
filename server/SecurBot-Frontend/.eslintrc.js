module.exports = {
  root: true,
  env: {
    node: true,
  },
  extends: [
    'plugin:vue/recommended',
    '@vue/airbnb',
  ],
  rules: {
    'no-console': process.env.NODE_ENV === 'production' ? 'error' : 'off',
    'no-debugger': process.env.NODE_ENV === 'production' ? 'error' : 'off',
    // Do not require extensions on import
    'import/extensions': ['warn', 'always', {
      js: 'never',
      vue: 'never'
    }],
    'no-param-reassign': ['warn', {
      props: true,
      ignorePropertyModificationsFor: [
        'state', // for vuex state
        'acc', // for reduce accumulators
        'e' // for e.returnvalue
      ]
    }],
    // allow optionalDependencies
    'import/no-extraneous-dependencies': ['warn', {
      optionalDependencies: ['test/unit/index.js']
    }],
    // Rule correction
    'no-plusplus': ["error", { "allowForLoopAfterthoughts": true }],
    "vue/html-closing-bracket-newline": ["error", {"singleline": "never","multiline": "never"}],
    'vue/name-property-casing': ["error", "kebab-case"],
    'no-unused-vars':'off',
    'no-restricted-syntax':'off',
  },
  parserOptions: {
    parser: 'babel-eslint',
  },
};
