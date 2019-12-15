FROM node:12

ENV NODE_ENV production
ENV API_PORT 3000

WORKDIR /opt/api

COPY ["package*.json", "./"]

RUN npm install --production --silent

COPY . .

EXPOSE 3000

# MongoDB waiter
# ADD https://github.com/ufoscout/docker-compose-wait/releases/download/2.6.0/wait ./wait
# RUN chmod +x ./wait

# CMD ./wait && npm start
CMD sleep 1 && npm start