--
-- PostgreSQL database dump
--

-- Dumped from database version 16.9 (Ubuntu 16.9-0ubuntu0.24.04.1)
-- Dumped by pg_dump version 16.9 (Ubuntu 16.9-0ubuntu0.24.04.1)

SET statement_timeout = 0;
SET lock_timeout = 0;
SET idle_in_transaction_session_timeout = 0;
SET client_encoding = 'UTF8';
SET standard_conforming_strings = on;
SELECT pg_catalog.set_config('search_path', '', false);
SET check_function_bodies = false;
SET xmloption = content;
SET client_min_messages = warning;
SET row_security = off;

--
-- Name: pgcrypto; Type: EXTENSION; Schema: -; Owner: -
--

CREATE EXTENSION IF NOT EXISTS pgcrypto WITH SCHEMA public;


--
-- Name: EXTENSION pgcrypto; Type: COMMENT; Schema: -; Owner: 
--

COMMENT ON EXTENSION pgcrypto IS 'cryptographic functions';


SET default_tablespace = '';

SET default_table_access_method = heap;

--
-- Name: hana_bots; Type: TABLE; Schema: public; Owner: postgres
--

CREATE TABLE public.hana_bots (
    hana_bot_id integer NOT NULL,
    bot_name character varying(10) NOT NULL,
    battery integer,
    status character varying,
    CONSTRAINT hana_bots_status_check CHECK (((status)::text = ANY ((ARRAY['충전중'::character varying, '작업중'::character varying, '대기중'::character varying, '복귀중'::character varying])::text[])))
);


ALTER TABLE public.hana_bots OWNER TO postgres;

--
-- Name: hana_bots_hana_bot_id_seq; Type: SEQUENCE; Schema: public; Owner: postgres
--

CREATE SEQUENCE public.hana_bots_hana_bot_id_seq
    AS integer
    START WITH 1
    INCREMENT BY 1
    NO MINVALUE
    NO MAXVALUE
    CACHE 1;


ALTER SEQUENCE public.hana_bots_hana_bot_id_seq OWNER TO postgres;

--
-- Name: hana_bots_hana_bot_id_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: postgres
--

ALTER SEQUENCE public.hana_bots_hana_bot_id_seq OWNED BY public.hana_bots.hana_bot_id;


--
-- Name: items; Type: TABLE; Schema: public; Owner: postgres
--

CREATE TABLE public.items (
    item_id integer NOT NULL,
    item_type character varying NOT NULL,
    item_quantity integer DEFAULT 100 NOT NULL,
    CONSTRAINT items_item_type_check CHECK (((item_type)::text = ANY ((ARRAY['물'::character varying, '영양제'::character varying, '생필품'::character varying, '식판'::character varying])::text[])))
);


ALTER TABLE public.items OWNER TO postgres;

--
-- Name: items_item_id_seq; Type: SEQUENCE; Schema: public; Owner: postgres
--

CREATE SEQUENCE public.items_item_id_seq
    AS integer
    START WITH 1
    INCREMENT BY 1
    NO MINVALUE
    NO MAXVALUE
    CACHE 1;


ALTER SEQUENCE public.items_item_id_seq OWNER TO postgres;

--
-- Name: items_item_id_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: postgres
--

ALTER SEQUENCE public.items_item_id_seq OWNED BY public.items.item_id;


--
-- Name: login_logs; Type: TABLE; Schema: public; Owner: postgres
--

CREATE TABLE public.login_logs (
    id integer NOT NULL,
    resident_id integer,
    success boolean,
    client_ip character varying(50),
    login_time timestamp without time zone DEFAULT CURRENT_TIMESTAMP
);


ALTER TABLE public.login_logs OWNER TO postgres;

--
-- Name: login_logs_id_seq; Type: SEQUENCE; Schema: public; Owner: postgres
--

CREATE SEQUENCE public.login_logs_id_seq
    AS integer
    START WITH 1
    INCREMENT BY 1
    NO MINVALUE
    NO MAXVALUE
    CACHE 1;


ALTER SEQUENCE public.login_logs_id_seq OWNER TO postgres;

--
-- Name: login_logs_id_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: postgres
--

ALTER SEQUENCE public.login_logs_id_seq OWNED BY public.login_logs.id;


--
-- Name: residents; Type: TABLE; Schema: public; Owner: postgres
--

CREATE TABLE public.residents (
    resident_id integer NOT NULL,
    name character varying(50) NOT NULL,
    gender character(1) NOT NULL,
    birth_date date NOT NULL,
    assigned_room_id integer,
    service_station_id integer,
    login_id character varying(50) NOT NULL,
    password character varying(255) DEFAULT public.crypt('1234'::text, public.gen_salt('bf'::text)) NOT NULL,
    CONSTRAINT residents_gender_check CHECK ((gender = ANY (ARRAY['M'::bpchar, 'F'::bpchar])))
);


ALTER TABLE public.residents OWNER TO postgres;

--
-- Name: residents_resident_id_seq; Type: SEQUENCE; Schema: public; Owner: postgres
--

CREATE SEQUENCE public.residents_resident_id_seq
    AS integer
    START WITH 1
    INCREMENT BY 1
    NO MINVALUE
    NO MAXVALUE
    CACHE 1;


ALTER SEQUENCE public.residents_resident_id_seq OWNER TO postgres;

--
-- Name: residents_resident_id_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: postgres
--

ALTER SEQUENCE public.residents_resident_id_seq OWNED BY public.residents.resident_id;


--
-- Name: hana_bots hana_bot_id; Type: DEFAULT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.hana_bots ALTER COLUMN hana_bot_id SET DEFAULT nextval('public.hana_bots_hana_bot_id_seq'::regclass);


--
-- Name: items item_id; Type: DEFAULT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.items ALTER COLUMN item_id SET DEFAULT nextval('public.items_item_id_seq'::regclass);


--
-- Name: login_logs id; Type: DEFAULT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.login_logs ALTER COLUMN id SET DEFAULT nextval('public.login_logs_id_seq'::regclass);


--
-- Name: residents resident_id; Type: DEFAULT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.residents ALTER COLUMN resident_id SET DEFAULT nextval('public.residents_resident_id_seq'::regclass);


--
-- Data for Name: hana_bots; Type: TABLE DATA; Schema: public; Owner: postgres
--

COPY public.hana_bots (hana_bot_id, bot_name, battery, status) FROM stdin;
7	HANABOT_7	\N	\N
8	HANABOT_8	\N	\N
9	HANABOT_9	\N	\N
10	HANA_PINKY	100	대기중
\.


--
-- Data for Name: items; Type: TABLE DATA; Schema: public; Owner: postgres
--

COPY public.items (item_id, item_type, item_quantity) FROM stdin;
9	물	100
10	영양제	100
11	생필품	100
12	식판	100
\.


--
-- Data for Name: login_logs; Type: TABLE DATA; Schema: public; Owner: postgres
--

COPY public.login_logs (id, resident_id, success, client_ip, login_time) FROM stdin;
1	1	t	127.0.0.1	2025-07-26 21:50:28.65401
2	\N	f	127.0.0.1	2025-07-26 22:07:01.334553
3	\N	f	127.0.0.1	2025-07-26 22:07:45.598618
4	\N	f	127.0.0.1	2025-07-26 22:07:48.718039
5	\N	f	127.0.0.1	2025-07-26 22:07:58.408657
6	2	t	127.0.0.1	2025-07-26 23:19:43.445885
7	2	t	127.0.0.1	2025-07-26 23:20:55.638729
8	1	t	127.0.0.1	2025-07-26 23:21:07.190418
10	1	t	127.0.0.1	2025-07-26 23:40:34.070484
11	\N	f	127.0.0.1	2025-07-26 23:40:43.927193
12	9999	t	127.0.0.1	2025-07-26 23:43:19.206874
13	9999	t	127.0.0.1	2025-07-26 23:44:17.40516
14	9999	t	127.0.0.1	2025-07-26 23:44:22.980135
18	9999	t	127.0.0.1	2025-07-26 23:47:42.697985
19	9999	t	127.0.0.1	2025-07-27 04:35:31.416605
20	9999	t	127.0.0.1	2025-07-27 04:51:35.367336
21	9999	t	127.0.0.1	2025-07-27 05:00:14.938507
22	9999	t	124.49.97.226	2025-07-27 05:10:39.667467
23	\N	f	192.168.219.115	2025-07-27 05:40:41.777295
24	9999	t	192.168.219.115	2025-07-27 05:40:44.891617
25	\N	f	192.168.219.111	2025-07-27 05:46:40.218719
26	9999	t	192.168.219.111	2025-07-27 05:46:51.442256
27	9999	t	127.0.0.1	2025-07-27 05:54:37.151836
28	9999	t	127.0.0.1	2025-07-27 06:00:23.820719
29	9999	t	127.0.0.1	2025-07-27 06:00:24.688395
\.


--
-- Data for Name: residents; Type: TABLE DATA; Schema: public; Owner: postgres
--

COPY public.residents (resident_id, name, gender, birth_date, assigned_room_id, service_station_id, login_id, password) FROM stdin;
2	김순자	F	1930-01-01	\N	\N	soon_30	$2a$06$wExDrXZHKyrQR90HXYyTSOqNGJeIO8KhfJLwVQwK0foOWf5QJgmhS
1	김복자	F	1940-01-01	\N	\N	40bokja	$2a$06$PTSh8ggRGKVxT1p5d0ZDl.glBkgOaRN7RWb3hX8L5pfMz8D7BR29O
9999	admin	M	1900-01-01	\N	\N	admin	$2a$06$BOFmpxmSteP7o5KQ2gfG4u/1hcr0m8xECPm7GQtQXW9GB/JwL13vi
\.


--
-- Name: hana_bots_hana_bot_id_seq; Type: SEQUENCE SET; Schema: public; Owner: postgres
--

SELECT pg_catalog.setval('public.hana_bots_hana_bot_id_seq', 10, true);


--
-- Name: items_item_id_seq; Type: SEQUENCE SET; Schema: public; Owner: postgres
--

SELECT pg_catalog.setval('public.items_item_id_seq', 13, true);


--
-- Name: login_logs_id_seq; Type: SEQUENCE SET; Schema: public; Owner: postgres
--

SELECT pg_catalog.setval('public.login_logs_id_seq', 29, true);


--
-- Name: residents_resident_id_seq; Type: SEQUENCE SET; Schema: public; Owner: postgres
--

SELECT pg_catalog.setval('public.residents_resident_id_seq', 2, true);


--
-- Name: hana_bots hana_bots_bot_name_key; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.hana_bots
    ADD CONSTRAINT hana_bots_bot_name_key UNIQUE (bot_name);


--
-- Name: hana_bots hana_bots_pkey; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.hana_bots
    ADD CONSTRAINT hana_bots_pkey PRIMARY KEY (hana_bot_id);


--
-- Name: items items_pkey; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.items
    ADD CONSTRAINT items_pkey PRIMARY KEY (item_id);


--
-- Name: login_logs login_logs_pkey; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.login_logs
    ADD CONSTRAINT login_logs_pkey PRIMARY KEY (id);


--
-- Name: residents residents_login_id_key; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.residents
    ADD CONSTRAINT residents_login_id_key UNIQUE (login_id);


--
-- Name: residents residents_pkey; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.residents
    ADD CONSTRAINT residents_pkey PRIMARY KEY (resident_id);


--
-- Name: login_logs fk_loginlogs_resident; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.login_logs
    ADD CONSTRAINT fk_loginlogs_resident FOREIGN KEY (resident_id) REFERENCES public.residents(resident_id);


--
-- PostgreSQL database dump complete
--

