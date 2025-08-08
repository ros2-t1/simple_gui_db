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
    bot_name character varying(50) NOT NULL,
    battery integer,
    status character varying,
    CONSTRAINT hana_bots_status_check CHECK (((status)::text = ANY (ARRAY['충전중'::text, '작업중'::text, '대기중'::text, '복귀중'::text, '오프라인'::text])))
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
-- Name: locations; Type: TABLE; Schema: public; Owner: postgres
--

CREATE TABLE public.locations (
    location_id integer NOT NULL,
    location_type character varying NOT NULL,
    location_name character varying(100) NOT NULL,
    coordinates double precision[],
    CONSTRAINT locations_location_type_check CHECK (((location_type)::text = ANY ((ARRAY['서비스 스테이션'::character varying, '픽업 스테이션'::character varying, '충전 스테이션'::character varying, '선반'::character varying])::text[])))
);


ALTER TABLE public.locations OWNER TO postgres;

--
-- Name: locations_location_id_seq; Type: SEQUENCE; Schema: public; Owner: postgres
--

CREATE SEQUENCE public.locations_location_id_seq
    AS integer
    START WITH 1
    INCREMENT BY 1
    NO MINVALUE
    NO MAXVALUE
    CACHE 1;


ALTER SEQUENCE public.locations_location_id_seq OWNER TO postgres;

--
-- Name: locations_location_id_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: postgres
--

ALTER SEQUENCE public.locations_location_id_seq OWNED BY public.locations.location_id;


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
-- Name: reserved_tasks; Type: TABLE; Schema: public; Owner: postgres
--

CREATE TABLE public.reserved_tasks (
    reserved_task_id integer NOT NULL,
    task_type character varying NOT NULL,
    requester_resident_id integer,
    item_id integer,
    target_location integer,
    scheduled_time timestamp without time zone NOT NULL,
    CONSTRAINT reserved_tasks_task_type_check CHECK (((task_type)::text = ANY ((ARRAY['식사 배달'::character varying, '물품 배달'::character varying])::text[])))
);


ALTER TABLE public.reserved_tasks OWNER TO postgres;

--
-- Name: reserved_tasks_reserved_task_id_seq; Type: SEQUENCE; Schema: public; Owner: postgres
--

CREATE SEQUENCE public.reserved_tasks_reserved_task_id_seq
    AS integer
    START WITH 1
    INCREMENT BY 1
    NO MINVALUE
    NO MAXVALUE
    CACHE 1;


ALTER SEQUENCE public.reserved_tasks_reserved_task_id_seq OWNER TO postgres;

--
-- Name: reserved_tasks_reserved_task_id_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: postgres
--

ALTER SEQUENCE public.reserved_tasks_reserved_task_id_seq OWNED BY public.reserved_tasks.reserved_task_id;


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
-- Name: tasks; Type: TABLE; Schema: public; Owner: postgres
--

CREATE TABLE public.tasks (
    task_id integer NOT NULL,
    task_type character varying NOT NULL,
    status character varying NOT NULL,
    requester_resident_id integer,
    item_id integer,
    target_location_id integer,
    source_reserved_task_id integer,
    assigned_bot_id integer,
    created_at timestamp without time zone DEFAULT CURRENT_TIMESTAMP NOT NULL,
    completed_at timestamp without time zone,
    CONSTRAINT tasks_status_check CHECK (((status)::text = ANY ((ARRAY['대기'::character varying, '할당'::character varying, '집기중'::character varying, '이동중'::character varying, '수령대기'::character varying, '완료'::character varying, '실패'::character varying])::text[]))),
    CONSTRAINT tasks_task_type_check CHECK (((task_type)::text = ANY ((ARRAY['배달'::character varying, '호출'::character varying])::text[])))
);


ALTER TABLE public.tasks OWNER TO postgres;

--
-- Name: tasks_task_id_seq; Type: SEQUENCE; Schema: public; Owner: postgres
--

CREATE SEQUENCE public.tasks_task_id_seq
    AS integer
    START WITH 1
    INCREMENT BY 1
    NO MINVALUE
    NO MAXVALUE
    CACHE 1;


ALTER SEQUENCE public.tasks_task_id_seq OWNER TO postgres;

--
-- Name: tasks_task_id_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: postgres
--

ALTER SEQUENCE public.tasks_task_id_seq OWNED BY public.tasks.task_id;


--
-- Name: hana_bots hana_bot_id; Type: DEFAULT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.hana_bots ALTER COLUMN hana_bot_id SET DEFAULT nextval('public.hana_bots_hana_bot_id_seq'::regclass);


--
-- Name: items item_id; Type: DEFAULT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.items ALTER COLUMN item_id SET DEFAULT nextval('public.items_item_id_seq'::regclass);


--
-- Name: locations location_id; Type: DEFAULT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.locations ALTER COLUMN location_id SET DEFAULT nextval('public.locations_location_id_seq'::regclass);


--
-- Name: login_logs id; Type: DEFAULT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.login_logs ALTER COLUMN id SET DEFAULT nextval('public.login_logs_id_seq'::regclass);


--
-- Name: reserved_tasks reserved_task_id; Type: DEFAULT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.reserved_tasks ALTER COLUMN reserved_task_id SET DEFAULT nextval('public.reserved_tasks_reserved_task_id_seq'::regclass);


--
-- Name: residents resident_id; Type: DEFAULT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.residents ALTER COLUMN resident_id SET DEFAULT nextval('public.residents_resident_id_seq'::regclass);


--
-- Name: tasks task_id; Type: DEFAULT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.tasks ALTER COLUMN task_id SET DEFAULT nextval('public.tasks_task_id_seq'::regclass);


--
-- Data for Name: hana_bots; Type: TABLE DATA; Schema: public; Owner: postgres
--

COPY public.hana_bots (hana_bot_id, bot_name, battery, status) FROM stdin;
10	HANA_PINKY	100	대기중
7	HANABOT_3	\N	오프라인
8	HANABOT_8	80	대기중
9	HANABOT_9	80	대기중
\.


--
-- Data for Name: items; Type: TABLE DATA; Schema: public; Owner: postgres
--

COPY public.items (item_id, item_type, item_quantity) FROM stdin;
12	식판	92
11	생필품	83
9	물	-2
10	영양제	46
\.


--
-- Data for Name: locations; Type: TABLE DATA; Schema: public; Owner: postgres
--

COPY public.locations (location_id, location_type, location_name, coordinates) FROM stdin;
1	서비스 스테이션	ROOM1	{0.1,0.78,-0.707}
3	충전 스테이션	CHARGER1	{0,0,1}
5	픽업 스테이션	PICKUP	{0.33,-0.33,-1}
2	서비스 스테이션	ROOM2	{0.3,0.66,-0.707}
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
30	9999	t	192.168.219.115	2025-07-27 08:39:34.504958
31	9999	t	192.168.219.115	2025-07-27 08:46:31.070564
32	9999	t	192.168.219.115	2025-07-27 11:02:48.812189
33	9999	t	192.168.219.115	2025-07-27 11:04:42.199085
34	9999	t	192.168.219.115	2025-07-27 11:06:15.105766
35	9999	t	127.0.0.1	2025-07-27 11:43:56.361823
36	2	t	127.0.0.1	2025-07-27 12:47:00.915778
37	\N	f	192.168.219.111	2025-07-27 12:55:29.295987
38	9999	t	192.168.219.111	2025-07-27 12:55:36.648054
39	\N	f	192.168.0.6	2025-07-28 01:24:26.248823
40	9999	t	192.168.0.6	2025-07-28 01:24:34.860888
41	2	t	192.168.0.19	2025-07-28 01:25:43.725823
42	9999	t	192.168.0.6	2025-07-28 03:18:16.262451
43	9999	t	192.168.0.7	2025-07-28 09:21:03.931387
44	\N	f	127.0.0.1	2025-07-28 09:24:02.546585
45	9999	t	127.0.0.1	2025-07-28 09:24:05.070081
46	9999	t	192.168.0.13	2025-07-28 09:25:27.511021
47	\N	f	192.168.0.13	2025-07-28 09:27:12.886656
48	\N	f	192.168.0.13	2025-07-28 09:27:17.758605
49	\N	f	192.168.0.13	2025-07-28 09:27:19.903993
50	\N	f	192.168.0.13	2025-07-28 09:27:21.318129
51	9999	t	192.168.0.13	2025-07-28 09:27:26.904852
52	9999	t	192.168.0.7	2025-07-29 10:28:11.807015
53	\N	f	192.168.0.7	2025-07-29 10:30:34.040631
54	9999	t	192.168.0.7	2025-07-29 10:37:17.7109
55	9999	t	192.168.0.7	2025-07-29 10:44:26.966008
56	9999	t	192.168.0.7	2025-07-29 10:46:00.208971
57	9999	t	192.168.0.7	2025-07-29 10:46:22.197675
58	9999	t	127.0.0.1	2025-07-29 12:49:10.356638
59	9999	t	127.0.0.1	2025-07-29 14:14:03.488422
60	9999	t	192.168.219.115	2025-07-29 14:28:45.141284
61	9999	t	192.168.219.115	2025-07-29 15:07:39.406453
62	9999	t	192.168.219.115	2025-07-29 15:31:43.067347
63	9999	t	192.168.219.111	2025-07-29 15:57:28.204878
64	9999	t	192.168.219.111	2025-07-29 16:19:57.260177
65	9999	t	192.168.0.13	2025-07-30 05:35:40.810894
66	9999	t	127.0.0.1	2025-07-30 06:30:19.201128
67	2	t	127.0.0.1	2025-07-30 07:47:33.749545
68	2	t	127.0.0.1	2025-07-30 11:47:42.406422
69	2	t	127.0.0.1	2025-07-30 12:27:49.301104
70	9999	t	127.0.0.1	2025-07-30 15:03:59.727216
71	9999	t	192.168.219.115	2025-07-30 15:10:01.832413
72	\N	f	192.168.219.111	2025-07-30 15:32:43.997459
73	9999	t	192.168.219.111	2025-07-30 15:32:47.272063
74	9999	t	192.168.219.111	2025-07-30 15:32:50.104494
75	9999	t	192.168.219.111	2025-07-30 15:32:54.742478
76	9999	t	192.168.219.111	2025-07-30 15:32:56.070971
77	9999	t	127.0.0.1	2025-07-30 15:37:17.53154
78	9999	t	127.0.0.1	2025-07-30 15:37:19.10759
79	9999	t	127.0.0.1	2025-07-30 15:37:19.285547
80	9999	t	127.0.0.1	2025-07-30 15:37:19.422349
81	9999	t	127.0.0.1	2025-07-30 15:37:19.562244
82	9999	t	127.0.0.1	2025-07-30 15:37:30.562544
83	2	t	192.168.0.13	2025-08-01 07:08:13.021897
84	9999	t	192.168.0.10	2025-08-06 02:43:08.028829
85	2	t	127.0.0.1	2025-08-06 04:05:54.387353
86	2	t	127.0.0.1	2025-08-06 04:06:12.572835
87	2	t	127.0.0.1	2025-08-06 04:10:05.922541
88	9999	t	127.0.0.1	2025-08-06 04:10:42.824429
89	9999	t	127.0.0.1	2025-08-06 04:13:15.511388
90	2	t	127.0.0.1	2025-08-06 04:15:08.963275
91	9999	t	127.0.0.1	2025-08-06 04:15:24.26417
92	2	t	192.168.0.7	2025-08-06 04:17:41.680229
93	9999	t	127.0.0.1	2025-08-06 04:30:10.511611
94	9999	t	127.0.0.1	2025-08-06 04:32:33.109217
95	9999	t	192.168.219.115	2025-08-06 17:56:46.04068
96	9999	t	127.0.0.1	2025-08-06 18:06:55.815248
97	9999	t	127.0.0.1	2025-08-06 18:21:49.370881
98	9999	t	192.168.0.10	2025-08-07 05:20:02.32714
\.


--
-- Data for Name: reserved_tasks; Type: TABLE DATA; Schema: public; Owner: postgres
--

COPY public.reserved_tasks (reserved_task_id, task_type, requester_resident_id, item_id, target_location, scheduled_time) FROM stdin;
\.


--
-- Data for Name: residents; Type: TABLE DATA; Schema: public; Owner: postgres
--

COPY public.residents (resident_id, name, gender, birth_date, assigned_room_id, service_station_id, login_id, password) FROM stdin;
1	김복자	F	1940-01-01	1	1	40bokja	$2a$06$PTSh8ggRGKVxT1p5d0ZDl.glBkgOaRN7RWb3hX8L5pfMz8D7BR29O
2	김순자	F	1930-01-01	2	2	soon_30	$2a$06$wExDrXZHKyrQR90HXYyTSOqNGJeIO8KhfJLwVQwK0foOWf5QJgmhS
9999	admin	M	1900-01-01	1	1	admin	$2a$06$BOFmpxmSteP7o5KQ2gfG4u/1hcr0m8xECPm7GQtQXW9GB/JwL13vi
\.


--
-- Data for Name: tasks; Type: TABLE DATA; Schema: public; Owner: postgres
--

COPY public.tasks (task_id, task_type, status, requester_resident_id, item_id, target_location_id, source_reserved_task_id, assigned_bot_id, created_at, completed_at) FROM stdin;
60	배달	완료	2	9	\N	\N	10	2025-07-30 12:33:03.636492	2025-07-30 12:34:09.564323
9	배달	완료	9999	9	\N	\N	10	2025-07-29 15:57:41.475438	2025-07-29 15:58:42.471243
11	배달	완료	9999	9	\N	\N	10	2025-07-29 16:03:56.247605	2025-07-29 16:04:40.397819
23	배달	완료	9999	9	\N	\N	10	2025-07-29 18:17:28.534815	2025-07-29 18:20:02.132412
35	배달	완료	9999	10	\N	\N	10	2025-07-30 06:38:35.098114	2025-07-30 06:39:09.889635
4	배달	완료	9999	\N	\N	\N	10	2025-07-29 15:41:15.405506	2025-07-29 15:42:50.362658
28	배달	실패	9999	9	\N	\N	10	2025-07-30 06:21:41.200103	2025-07-30 06:22:44.443735
16	배달	실패	9999	9	\N	\N	10	2025-07-29 17:13:17.0771	2025-07-29 17:18:42.864109
5	배달	완료	9999	\N	\N	\N	10	2025-07-29 15:41:22.686985	2025-07-29 15:42:54.274096
33	배달	완료	9999	9	\N	\N	10	2025-07-30 06:30:25.555726	2025-07-30 06:31:20.459021
13	배달	완료	9999	10	\N	\N	10	2025-07-29 16:10:29.076258	2025-07-29 16:12:51.029105
31	배달	실패	9999	9	\N	\N	10	2025-07-30 06:28:14.450461	2025-07-30 06:32:38.647567
134	호출	실패	9999	\N	\N	\N	\N	2025-08-06 11:21:44.322713	2025-08-06 11:22:25.947368
82	배달	실패	9999	9	\N	\N	10	2025-08-01 07:05:22.279157	2025-08-01 07:05:57.413187
26	배달	실패	9999	9	\N	\N	10	2025-07-30 00:43:15.569987	2025-07-30 00:44:27.774898
25	배달	실패	9999	9	\N	\N	10	2025-07-30 00:34:45.038622	2025-07-30 00:44:27.774898
80	호출	완료	2	\N	\N	\N	10	2025-07-30 12:55:38.90481	2025-07-30 12:55:42.487475
61	배달	완료	9999	10	\N	\N	10	2025-07-30 12:34:10.236192	2025-07-30 12:35:14.945668
47	배달	실패	9999	9	\N	\N	10	2025-07-30 08:48:44.424709	2025-07-30 08:50:28.10881
8	배달	완료	9999	\N	\N	\N	10	2025-07-29 15:50:40.818904	2025-07-29 15:52:20.031877
30	배달	완료	9999	10	\N	\N	10	2025-07-30 06:25:59.78522	2025-07-30 06:28:02.800489
81	배달	실패	9999	9	\N	\N	10	2025-08-01 07:05:13.113971	2025-08-01 07:05:57.413187
3	배달	완료	9999	\N	\N	\N	10	2025-07-29 15:35:37.88762	2025-07-29 15:40:30.241157
29	배달	실패	9999	9	\N	\N	10	2025-07-30 06:23:00.495997	2025-07-30 06:25:03.25255
48	배달	실패	2	9	\N	\N	10	2025-07-30 08:53:08.711409	2025-07-30 08:53:15.564055
10	배달	완료	9999	10	\N	\N	10	2025-07-29 15:58:02.176751	2025-07-29 15:59:26.922245
14	배달	완료	9999	9	\N	\N	10	2025-07-29 16:10:37.827045	2025-07-29 16:15:21.677167
15	배달	완료	9999	9	\N	\N	10	2025-07-29 16:20:06.351885	2025-07-29 16:22:23.21619
12	배달	실패	9999	10	\N	\N	10	2025-07-29 16:04:02.150444	2025-07-29 16:39:47.664703
18	배달	실패	9999	9	\N	\N	10	2025-07-29 17:27:08.167814	2025-07-29 17:28:25.53631
19	배달	실패	9999	9	\N	\N	\N	2025-07-29 17:29:09.029702	2025-07-29 17:30:19.686309
64	호출	완료	2	\N	\N	\N	10	2025-07-30 12:40:22.482167	2025-07-30 12:43:38.700037
83	배달	완료	9999	9	\N	\N	10	2025-08-01 07:06:05.259308	2025-08-01 07:06:34.113122
7	배달	완료	9999	\N	\N	\N	10	2025-07-29 15:43:57.005422	2025-07-29 15:46:50.012951
56	배달	완료	2	12	\N	\N	10	2025-07-30 12:10:21.139789	2025-07-30 12:11:59.426581
21	배달	완료	9999	9	\N	\N	10	2025-07-29 17:49:11.355123	2025-07-29 17:49:59.927517
43	배달	완료	2	9	\N	\N	10	2025-07-30 08:01:34.171523	2025-07-30 08:02:19.321385
57	배달	완료	2	10	\N	\N	10	2025-07-30 12:15:03.831175	2025-07-30 12:15:55.810799
6	배달	완료	9999	\N	\N	\N	10	2025-07-29 15:43:47.224697	2025-07-29 15:44:51.196828
39	배달	완료	2	9	\N	\N	10	2025-07-30 07:50:20.58155	2025-07-30 07:51:00.587196
46	배달	완료	2	9	\N	\N	10	2025-07-30 08:44:25.074514	2025-07-30 08:47:14.437749
58	배달	실패	2	9	\N	\N	10	2025-07-30 12:27:36.312744	2025-07-30 12:28:04.322705
24	배달	완료	9999	10	\N	\N	10	2025-07-29 18:18:23.400562	2025-07-29 18:21:05.321115
45	배달	실패	2	9	\N	\N	10	2025-07-30 08:36:41.573381	2025-07-30 08:37:15.206568
34	배달	완료	9999	9	\N	\N	10	2025-07-30 06:37:15.850573	2025-07-30 06:38:01.600349
52	배달	실패	2	9	\N	\N	10	2025-07-30 11:48:26.13454	2025-07-30 11:50:39.528226
59	배달	실패	2	9	\N	\N	10	2025-07-30 12:28:07.080355	2025-07-30 12:32:58.9041
32	배달	완료	9999	10	\N	\N	10	2025-07-30 06:28:23.157816	2025-07-30 06:30:30.19887
85	배달	완료	9999	10	\N	\N	10	2025-08-01 07:07:20.434311	2025-08-01 07:07:45.815856
40	배달	실패	2	10	\N	\N	10	2025-07-30 07:53:28.162383	2025-07-30 08:00:13.383624
41	배달	실패	2	9	\N	\N	10	2025-07-30 07:59:14.635203	2025-07-30 08:00:13.383624
42	배달	실패	9999	10	\N	\N	10	2025-07-30 07:59:16.645191	2025-07-30 08:00:13.383624
36	배달	완료	9999	11	\N	\N	10	2025-07-30 06:38:37.992596	2025-07-30 06:39:56.291719
86	배달	완료	9999	11	\N	\N	10	2025-08-01 07:08:06.652371	2025-08-01 07:08:34.360774
62	배달	완료	2	9	\N	\N	10	2025-07-30 12:36:56.460606	2025-07-30 12:37:46.562476
55	배달	완료	9999	9	\N	\N	10	2025-07-30 12:10:17.317834	2025-07-30 12:10:57.434364
27	배달	완료	9999	9	\N	\N	10	2025-07-30 00:44:52.420977	2025-07-30 00:46:07.783632
20	배달	완료	9999	9	\N	\N	10	2025-07-29 17:38:19.429736	2025-07-29 17:39:42.038697
17	배달	실패	9999	11	\N	\N	10	2025-07-29 17:13:47.123178	2025-07-29 17:22:57.570537
50	배달	실패	9999	10	\N	\N	\N	2025-07-30 11:38:39.526898	2025-07-30 11:39:59.539965
87	호출	완료	2	\N	\N	\N	10	2025-08-01 07:08:26.484712	2025-08-01 07:08:48.835494
38	배달	실패	2	9	\N	\N	10	2025-07-30 07:48:43.929599	2025-07-30 07:58:54.947151
22	배달	완료	9999	9	\N	\N	10	2025-07-29 18:16:24.733421	2025-07-29 18:17:23.441395
63	배달	완료	9999	10	\N	\N	10	2025-07-30 12:36:59.498311	2025-07-30 12:38:32.224153
88	배달	완료	9999	9	\N	\N	10	2025-08-05 11:36:54.247112	2025-08-05 11:37:17.36232
54	배달	완료	2	9	\N	\N	10	2025-07-30 12:09:10.474756	2025-07-30 12:09:55.164065
67	호출	완료	2	\N	\N	\N	10	2025-07-30 12:44:10.038238	2025-07-30 12:44:33.60806
37	배달	실패	2	9	\N	\N	10	2025-07-30 07:47:38.274029	2025-07-30 07:48:06.451993
65	호출	완료	2	\N	\N	\N	10	2025-07-30 12:43:36.896918	2025-07-30 12:43:42.425512
89	배달	완료	9999	9	\N	\N	10	2025-08-05 11:38:51.16625	2025-08-05 11:39:09.306924
51	배달	실패	2	9	\N	\N	10	2025-07-30 11:41:04.5273	2025-07-30 11:49:55.445209
44	배달	완료	9999	11	\N	\N	10	2025-07-30 08:01:37.459536	2025-07-30 08:03:05.685936
66	호출	완료	2	\N	\N	\N	10	2025-07-30 12:43:40.921148	2025-07-30 12:43:44.092841
49	배달	실패	2	9	\N	\N	10	2025-07-30 11:38:24.894352	2025-07-30 11:49:57.194857
53	배달	실패	2	9	\N	\N	10	2025-07-30 11:56:26.742939	2025-07-30 12:08:38.973712
90	배달	완료	2	10	\N	\N	10	2025-08-05 11:38:55.792773	2025-08-05 11:39:46.124439
72	호출	실패	9999	\N	\N	\N	10	2025-07-30 12:48:17.384425	2025-07-30 12:48:23.756723
91	배달	완료	9999	9	\N	\N	10	2025-08-06 02:43:17.865948	2025-08-06 02:43:40.426225
69	호출	완료	2	\N	\N	\N	10	2025-07-30 12:46:42.231779	2025-07-30 12:47:03.223579
92	배달	완료	9999	10	\N	\N	10	2025-08-06 02:46:05.804239	2025-08-06 02:51:29.187122
76	호출	완료	2	\N	\N	\N	10	2025-07-30 12:54:32.542193	2025-07-30 12:54:52.606113
68	배달	완료	2	9	\N	\N	10	2025-07-30 12:44:38.903642	2025-07-30 12:45:38.813287
73	호출	완료	9999	\N	\N	\N	10	2025-07-30 12:49:07.27708	2025-07-30 12:49:10.92081
70	호출	실패	9999	\N	\N	\N	10	2025-07-30 12:47:06.225846	2025-07-30 12:47:12.746441
93	배달	실패	9999	9	\N	\N	10	2025-08-06 02:53:11.554503	2025-08-06 03:02:38.131177
71	호출	완료	2	\N	\N	\N	10	2025-07-30 12:47:40.697788	2025-07-30 12:48:13.606842
84	배달	실패	9999	10	\N	\N	10	2025-08-01 07:06:35.907936	2025-08-06 03:01:53.984021
74	호출	완료	2	\N	\N	\N	10	2025-07-30 12:50:44.33402	2025-07-30 12:51:00.752295
95	배달	완료	9999	9	\N	\N	10	2025-08-06 03:00:47.22031	2025-08-06 03:01:27.151865
94	배달	실패	9999	9	\N	\N	10	2025-08-06 02:53:21.317769	2025-08-06 03:01:56.559721
79	호출	실패	2	\N	\N	\N	10	2025-07-30 12:55:21.973698	2025-07-30 12:55:28.323236
77	호출	실패	9999	\N	\N	\N	10	2025-07-30 12:54:54.787458	2025-07-30 12:55:01.317255
75	호출	실패	9999	\N	\N	\N	10	2025-07-30 12:51:03.523862	2025-07-30 12:51:10.253751
78	호출	완료	9999	\N	\N	\N	10	2025-07-30 12:55:09.245038	2025-07-30 12:55:16.619861
115	배달	실패	9999	9	\N	\N	\N	2025-08-06 05:06:09.713107	2025-08-06 05:07:00.660555
151	호출	실패	9999	\N	\N	\N	\N	2025-08-06 17:10:48.592288	2025-08-06 17:10:58.419044
137	배달	실패	9999	9	\N	\N	10	2025-08-06 11:27:43.636809	2025-08-06 11:30:45.346284
108	배달	완료	9999	9	\N	\N	10	2025-08-06 04:44:03.39972	2025-08-06 04:45:30.107886
120	배달	실패	9999	9	\N	\N	10	2025-08-06 05:23:40.75766	2025-08-06 05:24:28.820276
103	배달	실패	9999	9	\N	\N	10	2025-08-06 04:24:28.058483	2025-08-06 04:41:43.437098
101	배달	완료	2	9	\N	\N	10	2025-08-06 04:17:48.490466	2025-08-06 04:18:36.135776
131	배달	완료	9999	9	\N	\N	10	2025-08-06 06:44:15.574645	2025-08-06 06:44:35.051451
126	배달	완료	9999	9	\N	\N	10	2025-08-06 05:56:40.480964	2025-08-06 06:06:45.807822
109	호출	완료	9999	\N	\N	\N	10	2025-08-06 04:45:37.377451	2025-08-06 04:48:30.744946
178	호출	완료	9999	\N	\N	\N	8	2025-08-07 01:55:06.396013	2025-08-07 01:55:15.433113
142	배달	실패	9999	10	\N	\N	10	2025-08-06 11:59:03.263037	2025-08-06 11:59:30.405536
107	배달	실패	9999	11	\N	\N	10	2025-08-06 04:39:54.105306	2025-08-06 04:41:56.700582
102	배달	완료	9999	10	\N	\N	10	2025-08-06 04:19:02.10258	2025-08-06 04:19:20.044566
121	배달	완료	9999	9	\N	\N	10	2025-08-06 05:24:46.83212	2025-08-06 05:25:10.349165
105	배달	실패	9999	9	\N	\N	\N	2025-08-06 04:26:03.395079	2025-08-06 04:42:02.448206
138	배달	완료	9999	9	\N	\N	10	2025-08-06 11:30:33.184788	2025-08-06 11:31:10.530138
110	배달	완료	9999	9	\N	\N	10	2025-08-06 04:53:15.499988	2025-08-06 04:53:35.348495
172	배달	완료	9999	10	\N	\N	8	2025-08-07 01:32:13.152255	2025-08-07 01:33:13.424313
132	배달	완료	9999	12	\N	\N	10	2025-08-06 06:46:26.404723	2025-08-06 06:46:49.171283
104	배달	실패	9999	10	\N	\N	10	2025-08-06 04:24:39.481081	2025-08-06 04:43:51.219634
127	배달	완료	9999	9	\N	\N	10	2025-08-06 06:07:39.999561	2025-08-06 06:08:06.542797
169	배달	완료	9999	10	\N	\N	8	2025-08-07 01:16:16.17702	2025-08-07 01:17:23.557334
133	배달	실패	9999	9	\N	\N	10	2025-08-06 11:20:45.844671	2025-08-06 11:21:35.456168
135	호출	실패	9999	\N	\N	\N	\N	2025-08-06 11:21:50.730838	2025-08-06 11:22:27.001309
98	배달	완료	9999	9	\N	\N	10	2025-08-06 04:16:03.774702	2025-08-06 04:16:30.222705
97	배달	실패	9999	9	\N	\N	10	2025-08-06 04:10:47.281823	2025-08-06 04:17:05.349825
96	배달	실패	2	9	\N	\N	10	2025-08-06 04:10:19.612586	2025-08-06 04:17:09.164818
122	배달	완료	9999	9	\N	\N	10	2025-08-06 05:28:44.929308	2025-08-06 05:29:06.302727
119	배달	실패	9999	10	\N	\N	\N	2025-08-06 05:08:57.17238	2025-08-06 05:23:17.615743
145	배달	완료	9999	9	\N	\N	10	2025-08-06 16:44:00.521707	2025-08-06 16:44:38.049085
174	배달	완료	9999	10	\N	\N	8	2025-08-07 01:50:28.630478	2025-08-07 01:50:44.235238
111	배달	완료	9999	9	\N	\N	10	2025-08-06 04:55:10.303609	2025-08-06 04:55:30.557452
149	배달	실패	9999	10	\N	\N	10	2025-08-06 16:51:58.153651	2025-08-06 16:58:11.953142
158	배달	완료	9999	10	\N	\N	8	2025-08-06 17:55:48.875118	2025-08-06 18:20:17.363172
99	배달	완료	9999	9	\N	\N	10	2025-08-06 04:17:14.737903	2025-08-06 04:17:31.830354
117	배달	실패	2	9	\N	\N	10	2025-08-06 05:08:10.498291	2025-08-06 05:23:22.939506
154	배달	완료	9999	10	\N	\N	8	2025-08-06 17:20:42.525144	2025-08-06 17:21:32.994538
139	배달	완료	9999	9	\N	\N	10	2025-08-06 11:32:19.043822	2025-08-06 11:33:05.046113
123	배달	완료	2	10	\N	\N	10	2025-08-06 05:28:46.413322	2025-08-06 05:29:26.053958
125	배달	실패	9999	9	\N	\N	10	2025-08-06 05:55:52.021376	2025-08-06 06:04:14.811671
124	호출	완료	9999	\N	\N	\N	10	2025-08-06 05:29:20.582076	2025-08-06 05:29:32.49208
112	배달	완료	2	10	\N	\N	10	2025-08-06 04:57:58.864792	2025-08-06 04:58:32.322532
128	배달	완료	9999	9	\N	\N	10	2025-08-06 06:08:01.075813	2025-08-06 06:08:50.620328
113	배달	실패	9999	9	\N	\N	10	2025-08-06 05:04:01.116188	2025-08-06 05:05:08.612058
114	배달	실패	9999	9	\N	\N	\N	2025-08-06 05:04:29.622119	2025-08-06 05:05:15.252804
100	배달	완료	9999	9	\N	\N	10	2025-08-06 04:17:46.586852	2025-08-06 04:18:16.688064
116	배달	실패	9999	9	\N	\N	10	2025-08-06 05:07:06.429989	2025-08-06 05:19:55.290207
136	배달	완료	9999	9	\N	\N	10	2025-08-06 11:25:10.201324	2025-08-06 11:25:54.22868
140	배달	완료	9999	9	\N	\N	10	2025-08-06 11:33:44.568153	2025-08-06 11:34:41.921606
144	배달	실패	9999	9	\N	\N	10	2025-08-06 16:13:07.546933	2025-08-06 16:43:11.924554
129	배달	완료	9999	10	\N	\N	10	2025-08-06 06:39:28.16593	2025-08-06 06:43:44.322778
118	배달	실패	9999	9	\N	\N	10	2025-08-06 05:08:47.56314	2025-08-06 05:24:18.9034
130	호출	완료	9999	\N	\N	\N	10	2025-08-06 06:43:40.996931	2025-08-06 06:43:48.191953
141	배달	완료	9999	9	\N	\N	10	2025-08-06 11:35:00.073825	2025-08-06 11:36:15.695047
167	호출	실패	9999	\N	\N	\N	8	2025-08-07 01:10:08.561061	2025-08-07 01:14:57.694496
170	호출	완료	9999	\N	\N	\N	9	2025-08-07 01:16:20.149839	2025-08-07 01:18:29.563168
179	배달	실패	9999	10	\N	\N	\N	2025-08-07 05:21:02.976556	2025-08-07 05:28:42.652956
146	배달	완료	9999	9	\N	\N	10	2025-08-06 16:46:01.773678	2025-08-06 16:46:39.193857
152	배달	완료	9999	10	\N	\N	8	2025-08-06 17:11:04.095139	2025-08-06 17:11:47.683333
155	배달	완료	9999	10	\N	\N	8	2025-08-06 17:28:29.059404	2025-08-06 17:29:10.560526
173	호출	완료	9999	\N	\N	\N	9	2025-08-07 01:32:20.319643	2025-08-07 01:33:25.624405
175	호출	완료	9999	\N	\N	\N	9	2025-08-07 01:50:34.538975	2025-08-07 01:50:47.841751
171	배달	완료	9999	10	\N	\N	8	2025-08-07 01:16:53.253141	2025-08-07 01:18:27.046118
180	호출	실패	9999	\N	\N	\N	\N	2025-08-07 05:24:38.039987	2025-08-07 05:28:42.652956
176	배달	완료	9999	10	\N	\N	8	2025-08-07 01:52:07.062503	2025-08-07 01:52:22.516955
147	배달	완료	9999	10	\N	\N	10	2025-08-06 16:46:34.902075	2025-08-06 16:47:16.027538
153	배달	완료	9999	10	\N	\N	8	2025-08-06 17:18:42.744904	2025-08-06 17:19:38.213349
168	배달	실패	9999	10	\N	\N	9	2025-08-07 01:12:20.928161	2025-08-07 01:14:25.775544
156	배달	완료	9999	10	\N	\N	8	2025-08-06 17:32:32.617506	2025-08-06 17:55:15.41438
181	호출	대기	9999	\N	\N	\N	\N	2025-08-07 05:28:46.203405	\N
157	호출	실패	9999	\N	\N	\N	9	2025-08-06 17:32:44.090962	2025-08-06 17:32:49.581827
177	호출	완료	9999	\N	\N	\N	9	2025-08-07 01:52:07.976568	2025-08-07 01:52:34.564722
148	배달	완료	9999	9	\N	\N	10	2025-08-06 16:50:32.154014	2025-08-06 16:51:32.51309
150	호출	실패	9999	\N	\N	\N	8	2025-08-06 16:58:27.63	2025-08-06 17:10:58.419044
143	배달	실패	9999	9	\N	\N	8	2025-08-06 15:37:17.000143	2025-08-06 15:45:32.450467
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
-- Name: locations_location_id_seq; Type: SEQUENCE SET; Schema: public; Owner: postgres
--

SELECT pg_catalog.setval('public.locations_location_id_seq', 1, false);


--
-- Name: login_logs_id_seq; Type: SEQUENCE SET; Schema: public; Owner: postgres
--

SELECT pg_catalog.setval('public.login_logs_id_seq', 98, true);


--
-- Name: reserved_tasks_reserved_task_id_seq; Type: SEQUENCE SET; Schema: public; Owner: postgres
--

SELECT pg_catalog.setval('public.reserved_tasks_reserved_task_id_seq', 1, false);


--
-- Name: residents_resident_id_seq; Type: SEQUENCE SET; Schema: public; Owner: postgres
--

SELECT pg_catalog.setval('public.residents_resident_id_seq', 2, true);


--
-- Name: tasks_task_id_seq; Type: SEQUENCE SET; Schema: public; Owner: postgres
--

SELECT pg_catalog.setval('public.tasks_task_id_seq', 181, true);


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
-- Name: locations locations_location_name_key; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.locations
    ADD CONSTRAINT locations_location_name_key UNIQUE (location_name);


--
-- Name: locations locations_pkey; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.locations
    ADD CONSTRAINT locations_pkey PRIMARY KEY (location_id);


--
-- Name: login_logs login_logs_pkey; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.login_logs
    ADD CONSTRAINT login_logs_pkey PRIMARY KEY (id);


--
-- Name: reserved_tasks reserved_tasks_pkey; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.reserved_tasks
    ADD CONSTRAINT reserved_tasks_pkey PRIMARY KEY (reserved_task_id);


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
-- Name: tasks tasks_pkey; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.tasks
    ADD CONSTRAINT tasks_pkey PRIMARY KEY (task_id);


--
-- Name: idx_rtasks_sched_time; Type: INDEX; Schema: public; Owner: postgres
--

CREATE INDEX idx_rtasks_sched_time ON public.reserved_tasks USING btree (scheduled_time);


--
-- Name: idx_tasks_created_at; Type: INDEX; Schema: public; Owner: postgres
--

CREATE INDEX idx_tasks_created_at ON public.tasks USING btree (created_at);


--
-- Name: idx_tasks_status; Type: INDEX; Schema: public; Owner: postgres
--

CREATE INDEX idx_tasks_status ON public.tasks USING btree (status);


--
-- Name: login_logs fk_loginlogs_resident; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.login_logs
    ADD CONSTRAINT fk_loginlogs_resident FOREIGN KEY (resident_id) REFERENCES public.residents(resident_id);


--
-- Name: residents fk_service_station; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.residents
    ADD CONSTRAINT fk_service_station FOREIGN KEY (service_station_id) REFERENCES public.locations(location_id);


--
-- Name: reserved_tasks reserved_tasks_item_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.reserved_tasks
    ADD CONSTRAINT reserved_tasks_item_id_fkey FOREIGN KEY (item_id) REFERENCES public.items(item_id);


--
-- Name: reserved_tasks reserved_tasks_requester_resident_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.reserved_tasks
    ADD CONSTRAINT reserved_tasks_requester_resident_id_fkey FOREIGN KEY (requester_resident_id) REFERENCES public.residents(resident_id);


--
-- Name: reserved_tasks reserved_tasks_target_location_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.reserved_tasks
    ADD CONSTRAINT reserved_tasks_target_location_fkey FOREIGN KEY (target_location) REFERENCES public.locations(location_id);


--
-- Name: tasks tasks_assigned_bot_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.tasks
    ADD CONSTRAINT tasks_assigned_bot_id_fkey FOREIGN KEY (assigned_bot_id) REFERENCES public.hana_bots(hana_bot_id);


--
-- Name: tasks tasks_item_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.tasks
    ADD CONSTRAINT tasks_item_id_fkey FOREIGN KEY (item_id) REFERENCES public.items(item_id);


--
-- Name: tasks tasks_requester_resident_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.tasks
    ADD CONSTRAINT tasks_requester_resident_id_fkey FOREIGN KEY (requester_resident_id) REFERENCES public.residents(resident_id);


--
-- Name: tasks tasks_source_reserved_task_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.tasks
    ADD CONSTRAINT tasks_source_reserved_task_id_fkey FOREIGN KEY (source_reserved_task_id) REFERENCES public.reserved_tasks(reserved_task_id);


--
-- Name: tasks tasks_target_location_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.tasks
    ADD CONSTRAINT tasks_target_location_id_fkey FOREIGN KEY (target_location_id) REFERENCES public.locations(location_id);


--
-- PostgreSQL database dump complete
--

