FROM node:12 as build-stage
WORKDIR /app
COPY ["package.json", "package-lock.json*", "./"]
RUN npm install
COPY . .
RUN npm run build

FROM nginx:stable-alpine as production-stage
COPY --from=build-stage /app/dist /usr/share/nginx/app
COPY nginx.conf /etc/nginx
# docker run -i -t --volumes-from frontend --name frontend-files debian /bin/bash
VOLUME [ "/var/log/nginx" ]
CMD ["nginx", "-g", "daemon off;"]