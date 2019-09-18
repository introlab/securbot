# Frontend

- [Frontend](#frontend)
  - [Summary](#summary)
  - [Initial step to setup the project](#initial-step-to-setup-the-project)
    - [Compiles and hot-reloads for development](#compiles-and-hot-reloads-for-development)
    - [Compiles and minifies for production](#compiles-and-minifies-for-production)
    - [Generates the frontend documentation](#generates-the-frontend-documentation)
    - [Lints and fixes files](#lints-and-fixes-files)
    - [Run your tests](#run-your-tests)
    - [Run your end-to-end tests](#run-your-end-to-end-tests)
    - [Run your unit tests](#run-your-unit-tests)

## Summary

The frontend is been develop using the Vue.js framework with bootstrap toolbox and so is web base.
The interface is there to offer an easy way for operators to connect to any robot in services and perform some specific actions on them.

> We currently do not support mobile devices.

## Initial step to setup the project
```sh
npm install
```

### Compiles and hot-reloads for development
```sh
npm run serve
```

### Compiles and minifies for production
```sh
npm run build
```

### Generates the frontend documentation
```sh
npm run docs
```

### Lints and fixes files
> ESLint is use for linting the code.
```sh
npm run lint
```

### Run your tests
> There is currently no test suits even though the packages (Mocha + Nightwatch) are included in dependcies.
```sh
npm run test
```

### Run your end-to-end tests
> Only runs the e2e (Nightwatch) test suits.
```sh
npm run test:e2e
```

### Run your unit tests
> Only runs the unit (mocha) test suits.
```sh
npm run test:unit
```
