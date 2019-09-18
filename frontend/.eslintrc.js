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
    'vue/name-property-casing': ["error", "kebab-case"],
    'no-restricted-syntax':'off', // To disallow some javascript syntax, like some for of the for operator
  },
  parserOptions: {
    parser: 'babel-eslint',
  },
};
