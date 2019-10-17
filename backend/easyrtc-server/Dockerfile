FROM node:12

ENV NODE_ENV production
ENV PORT 8080

WORKDIR /opt/web_rtc

COPY ["package.json", "package-lock.json*", "./"]

RUN npm install --production --silent

COPY . .

EXPOSE 8080

CMD npm start